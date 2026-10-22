#ifndef AES_CRYPTO_H
#define AES_CRYPTO_H

#include <Arduino.h>
#include "mbedtls/aes.h"
#include "mbedtls/md.h"
#include "esp_system.h"
#include "private_config.h"

// HMAC key is derived with a fixed label so it is separate from the AES key.
static const char MAC_KEY_LABEL[] = "SanMiguel LoRa CBC MAC v1";
static const size_t HMAC_SIZE = 32; // full SHA-256 output; frame tags use a truncated prefix
static const size_t MAX_ENCRYPTED_PACKET_SIZE = 250; // Thesis LoRa MTU limit

class AESCrypto {
private:
    static bool computeMacKey(uint8_t *macKey) {
        const mbedtls_md_info_t *mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        if (!mdInfo) return false;
        return mbedtls_md_hmac(mdInfo, PRE_SHARED_KEY, sizeof(PRE_SHARED_KEY),
                               reinterpret_cast<const unsigned char *>(MAC_KEY_LABEL),
                               sizeof(MAC_KEY_LABEL) - 1, macKey) == 0;
    }

    static bool computeHmac(const uint8_t *data, size_t dataLen, uint8_t *tag) {
        const mbedtls_md_info_t *mdInfo = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        if (!mdInfo) return false;
        uint8_t macKey[HMAC_SIZE];
        if (!computeMacKey(macKey)) return false;
        int ret = mbedtls_md_hmac(mdInfo, macKey, sizeof(macKey), data, dataLen, tag);
        memset(macKey, 0, sizeof(macKey));
        return ret == 0;
    }

    static bool tagsMatch(const uint8_t *a, const uint8_t *b, size_t len) {
        uint8_t difference = 0;
        for (size_t i = 0; i < len; i++) difference |= a[i] ^ b[i];
        return difference == 0;
    }

public:
    // Generate cryptographically random 16-byte IV
    static void generateIV(uint8_t *iv) {
        esp_fill_random(iv, 16);
    }

    // Encrypt with AES-128-CBC/PKCS#7.
    // Output format: [16-byte IV] + [Ciphertext]. Integrity and authenticity are
    // provided once per hop by the truncated HMAC frame tag (authenticateFrame),
    // which covers the packet header, the IV and the ciphertext together.
    static bool encrypt(const uint8_t *plain, size_t plainLen, uint8_t *outBuffer, size_t &outLen, size_t maxOutLen) {
        outLen = 0;
        if (!plain || !outBuffer || plainLen == 0 || plainLen > 240) return false;

        // PKCS#7 padding calculation
        uint8_t padVal = 16 - (plainLen % 16);
        size_t paddedLen = plainLen + padVal;
        size_t totalRequired = 16 + paddedLen;

        if (totalRequired > maxOutLen || totalRequired > MAX_ENCRYPTED_PACKET_SIZE) return false;

        uint8_t iv[16];
        generateIV(iv);
        memcpy(outBuffer, iv, 16); // Prepend IV

        uint8_t tempBuf[256];
        memcpy(tempBuf, plain, plainLen);
        memset(tempBuf + plainLen, padVal, padVal);

        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);
        mbedtls_aes_setkey_enc(&aes, PRE_SHARED_KEY, 128);

        uint8_t iv_copy[16];
        memcpy(iv_copy, iv, 16); // mbedtls mutates IV during CBC

        int ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_ENCRYPT, paddedLen, iv_copy, tempBuf, outBuffer + 16);
        mbedtls_aes_free(&aes);
        memset(tempBuf, 0, sizeof(tempBuf));

        if (ret != 0) return false;
        outLen = totalRequired;
        return true;
    }

    // Decrypt [IV + ciphertext]. Call only after the frame tag has been verified.
    // outPlain receives binary data; one extra NUL byte is written after it.
    static bool decrypt(const uint8_t *inBuffer, size_t inLen, uint8_t *outPlain, size_t &outPlainLen, size_t maxOutLen) {
        outPlainLen = 0;
        if (!inBuffer || !outPlain || inLen < (16 + 16) || inLen > MAX_ENCRYPTED_PACKET_SIZE) return false;

        size_t cipherLen = inLen - 16;
        if ((cipherLen % 16) != 0 || cipherLen > 256) return false;

        uint8_t iv[16];
        memcpy(iv, inBuffer, 16);

        uint8_t tempBuf[256];
        mbedtls_aes_context aes;
        mbedtls_aes_init(&aes);
        mbedtls_aes_setkey_dec(&aes, PRE_SHARED_KEY, 128);

        int ret = mbedtls_aes_crypt_cbc(&aes, MBEDTLS_AES_DECRYPT, cipherLen, iv, inBuffer + 16, tempBuf);
        mbedtls_aes_free(&aes);

        if (ret != 0) return false;

        // PKCS#7 unpadding
        uint8_t padVal = tempBuf[cipherLen - 1];
        if (padVal == 0 || padVal > 16) return false;
        for (size_t i = 0; i < padVal; i++) {
            if (tempBuf[cipherLen - 1 - i] != padVal) return false;
        }

        size_t actualLen = cipherLen - padVal;
        if (actualLen >= maxOutLen) return false;

        memcpy(outPlain, tempBuf, actualLen);
        outPlain[actualLen] = 0;
        outPlainLen = actualLen;
        memset(tempBuf, 0, sizeof(tempBuf));
        return true;
    }

    // Validate Community Credential
    static bool verifyCommunityCredential(const char *credential) {
        if (!credential) return false;
        return (strcmp(credential, COMMUNITY_ACCESS_CREDENTIAL) == 0);
    }

    // Writes the first tagLen bytes (1..32) of HMAC-SHA-256(data) to tag.
    static bool authenticateFrame(const uint8_t *data, size_t dataLen, uint8_t *tag, size_t tagLen) {
        if (!data || !tag || tagLen == 0 || tagLen > HMAC_SIZE) return false;
        uint8_t fullTag[HMAC_SIZE];
        bool ok = computeHmac(data, dataLen, fullTag);
        if (ok) memcpy(tag, fullTag, tagLen);
        memset(fullTag, 0, sizeof(fullTag));
        return ok;
    }

    // Constant-time check of a truncated tag produced by authenticateFrame.
    static bool verifyFrame(const uint8_t *data, size_t dataLen, const uint8_t *tag, size_t tagLen) {
        if (!data || !tag || tagLen == 0 || tagLen > HMAC_SIZE) return false;
        uint8_t expectedTag[HMAC_SIZE];
        if (!computeHmac(data, dataLen, expectedTag)) return false;
        bool matches = tagsMatch(expectedTag, tag, tagLen);
        memset(expectedTag, 0, sizeof(expectedTag));
        return matches;
    }
};

#endif
