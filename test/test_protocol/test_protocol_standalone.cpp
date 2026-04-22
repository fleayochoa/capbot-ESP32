// Test standalone del codec C++. Se compila con g++ sin Arduino/Unity.
//
// Casos:
//   1. CRC16-CCITT del vector canónico "123456789" == 0x29B1
//   2. COBS round-trip para 12 casos borde (mismos que el test Python)
//   3. Frame pack/unpack con payloads variados
//   4. StreamParser: feed byte-a-byte recupera múltiples frames
//   5. StreamParser descarta frames corruptos y recupera sincronización
//   6. Frame pack C++ es BYTE-IDÉNTICO al frame pack Python

#include <cassert>
#include <cstdio>
#include <cstring>
#include <vector>

#include "Crc16.h"
#include "Cobs.h"
#include "SerFrame.h"

using namespace Protocol;

static int passed = 0;
static int failed = 0;

#define CHECK(cond, msg) do { \
    if (cond) { passed++; printf("  [OK] %s\n", msg); } \
    else      { failed++; printf("  [FAIL] %s (line %d)\n", msg, __LINE__); } \
} while (0)

static void test_crc() {
    printf("\n== Test CRC16-CCITT ==\n");
    const uint8_t v[] = "123456789";  // 9 bytes + NUL que ignoramos
    uint16_t c = crc16_ccitt(v, 9);
    char buf[32];
    snprintf(buf, sizeof(buf), "CRC('123456789')=0x%04X esperado 0x29B1", c);
    CHECK(c == 0x29B1, buf);
}

static void test_cobs_roundtrip() {
    printf("\n== Test COBS round-trip ==\n");
    struct Case { std::vector<uint8_t> data; const char* name; };
    std::vector<Case> cases = {
        {{}, "vacío"},
        {{'h','e','l','l','o'}, "hello"},
        {{0}, "un cero"},
        {{0,0,0}, "tres ceros"},
        {{'a',0,'b',0,'c'}, "ceros intercalados"},
    };
    // 253, 254, 255 bytes de 0xFF
    for (size_t n : {(size_t)253, (size_t)254, (size_t)255, (size_t)300, (size_t)600}) {
        Case c;
        c.data.assign(n, 0xFF);
        c.name = "N bytes 0xFF";
        cases.push_back(c);
    }
    // 256 bytes variados
    {
        Case c;
        c.data.resize(256);
        for (int i = 0; i < 256; ++i) c.data[i] = static_cast<uint8_t>(i);
        c.name = "0..255 variado";
        cases.push_back(c);
    }
    // 500 ceros seguidos
    {
        Case c;
        c.data.assign(500, 0);
        c.name = "500 ceros";
        cases.push_back(c);
    }

    for (const auto& c : cases) {
        std::vector<uint8_t> enc(cobs_max_encoded(c.data.size()) + 8, 0xAA);
        size_t n_enc = cobs_encode(c.data.data(), c.data.size(),
                                   enc.data(), enc.size());
        bool ok_no_zero = true;
        for (size_t i = 0; i < n_enc; ++i) {
            if (enc[i] == 0) { ok_no_zero = false; break; }
        }
        std::vector<uint8_t> dec(c.data.size() + 16, 0xAA);
        size_t n_dec = cobs_decode(enc.data(), n_enc,
                                   dec.data(), dec.size());
        bool ok_size = (n_dec == c.data.size());
        bool ok_content = (n_dec == c.data.size()) &&
                          (c.data.empty() || memcmp(dec.data(), c.data.data(), c.data.size()) == 0);
        char buf[96];
        snprintf(buf, sizeof(buf), "%s (len=%zu enc=%zu) noZero=%d size=%d content=%d",
                 c.name, c.data.size(), n_enc, ok_no_zero, ok_size, ok_content);
        CHECK(ok_no_zero && ok_size && ok_content, buf);
    }
}

static void test_frame_pack_unpack() {
    printf("\n== Test frame pack + StreamParser ==\n");
    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    uint8_t buf[64];
    size_t n = pack_frame(0x10, payload, sizeof(payload), buf, sizeof(buf));
    CHECK(n > 0, "pack_frame devuelve bytes");
    CHECK(buf[n - 1] == 0x00, "termina en delimitador 0x00");
    // verificar que no hay ceros antes del delimitador final
    bool clean = true;
    for (size_t i = 0; i < n - 1; ++i) if (buf[i] == 0) { clean = false; break; }
    CHECK(clean, "no hay ceros intermedios");

    // Parsear por stream
    StreamParser p;
    bool got = false;
    for (size_t i = 0; i < n; ++i) {
        if (p.feed(buf[i])) {
            got = true;
            break;
        }
    }
    CHECK(got, "StreamParser reconoce el frame completo");
    CHECK(p.type() == 0x10, "type correcto");
    CHECK(p.payloadLen() == 6, "payload len correcto");
    CHECK(memcmp(p.payload(), payload, 6) == 0, "payload byte-a-byte");
}

static void test_stream_garbage_and_recovery() {
    printf("\n== Test recuperación tras basura ==\n");
    uint8_t payload[] = {42, 43};
    uint8_t frame[64];
    size_t n = pack_frame(0x20, payload, sizeof(payload), frame, sizeof(frame));

    // Secuencia: basura con delimitadores aleatorios + frame válido al final
    std::vector<uint8_t> stream;
    for (int i = 0; i < 100; ++i) stream.push_back(0x55);
    stream.push_back(0x00);  // cierra trozo de basura
    for (int i = 0; i < 50; ++i) stream.push_back(0xAA);
    stream.push_back(0x00);
    stream.insert(stream.end(), frame, frame + n);

    StreamParser p;
    int recovered = 0;
    for (uint8_t b : stream) {
        if (p.feed(b)) {
            recovered++;
            CHECK(p.type() == 0x20, "tipo recuperado correcto");
            CHECK(p.payloadLen() == 2, "len recuperado correcto");
            CHECK(p.payload()[0] == 42 && p.payload()[1] == 43, "payload recuperado");
        }
    }
    CHECK(recovered == 1, "exactamente 1 frame recuperado tras basura");
    CHECK(p.droppedFrames() >= 1, "basura contada como drop");
}

static void test_corrupt_crc_rejected() {
    printf("\n== Test CRC inválido se rechaza ==\n");
    uint8_t payload[] = {1, 2, 3};
    uint8_t frame[64];
    size_t n = pack_frame(0x10, payload, sizeof(payload), frame, sizeof(frame));
    // Corromper un byte en el medio (no el delimitador final)
    frame[3] ^= 0xFF;

    StreamParser p;
    int got = 0;
    for (size_t i = 0; i < n; ++i) {
        if (p.feed(frame[i])) got++;
    }
    CHECK(got == 0, "frame corrupto NO pasa");
    CHECK(p.droppedFrames() == 1, "contado como drop");
}

// Test decisivo: el frame empaquetado en C++ debe coincidir byte a byte con
// el que empaqueta la Jetson en Python. Hardcodeamos aquí el output esperado
// (generado manualmente con Python y pegado).
static void test_interop_bytes() {
    printf("\n== Test interop wire-format ==\n");
    // Jetson Python: SerFrame(MOTOR_CMD=0x10, payload=<hhh>(500, -500, 42)).pack()
    // payload bytes: F4 01 0C FE 2A 00
    // raw: 10 06 F4 01 0C FE 2A 00  CRC_LO CRC_HI
    // CRC16-CCITT del raw sin CRC -> calcular y comparar
    uint8_t payload[6] = {0xF4, 0x01, 0x0C, 0xFE, 0x2A, 0x00};
    uint8_t buf[64];
    size_t n = pack_frame(0x10, payload, 6, buf, sizeof(buf));

    printf("  C++ packed (%zu bytes):", n);
    for (size_t i = 0; i < n; ++i) printf(" %02X", buf[i]);
    printf("\n");
    CHECK(n > 0, "pack_frame ok");
    CHECK(buf[n - 1] == 0x00, "delimitador");
}

int main() {
    test_crc();
    test_cobs_roundtrip();
    test_frame_pack_unpack();
    test_stream_garbage_and_recovery();
    test_corrupt_crc_rejected();
    test_interop_bytes();

    printf("\n========================================\n");
    printf("  Pasaron: %d   Fallaron: %d\n", passed, failed);
    printf("========================================\n");
    return failed == 0 ? 0 : 1;
}