/**
 * Header file for eCryptfs userspace tools.
 *
 * Copyright (C) 2004-2008 International Business Machines Corp.
 *   Author(s): Michael A. Halcrow <mahalcro@us.ibm.com>
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * The structs here are shared between kernel and userspace, so if you
 * are running a 64-bit kernel, you need to compile your userspace
 * applications as 64-bit binaries.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */

#include <string.h>
#include <errno.h>
#include <openssl/sha.h>

#define VERSION_MAJOR 0x00
#define VERSION_MINOR 0x04
#define ECRYPTFS_SIG_SIZE 8
#define ECRYPTFS_SIG_SIZE_HEX (ECRYPTFS_SIG_SIZE*2)
#define ECRYPTFS_PASSWORD_SIG_SIZE ECRYPTFS_SIG_SIZE_HEX
#define ECRYPTFS_SALT_SIZE 8
#define ECRYPTFS_MAX_ENCRYPTED_KEY_BYTES 512
#define ECRYPTFS_MAX_KEY_BYTES 64
#define ECRYPTFS_MAX_PASSWORD_LENGTH 64
#define ECRYPTFS_MAX_PASSPHRASE_BYTES ECRYPTFS_MAX_PASSWORD_LENGTH
#define PGP_DIGEST_ALGO_SHA512 10

/* Hash iterations are intended to make dictionary attacks more difficult */
#define ECRYPTFS_DEFAULT_NUM_HASH_ITERATIONS 65536

struct ecryptfs_session_key {
#define ECRYPTFS_USERSPACE_SHOULD_TRY_TO_DECRYPT 0x00000001
#define ECRYPTFS_USERSPACE_SHOULD_TRY_TO_ENCRYPT 0x00000002
#define ECRYPTFS_CONTAINS_DECRYPTED_KEY 0x00000004
#define ECRYPTFS_CONTAINS_ENCRYPTED_KEY 0x00000008
    int32_t flags;
    int32_t encrypted_key_size;
    int32_t decrypted_key_size;
    uint8_t encrypted_key[ECRYPTFS_MAX_ENCRYPTED_KEY_BYTES];
    uint8_t decrypted_key[ECRYPTFS_MAX_KEY_BYTES];
};

struct ecryptfs_password {
    int32_t password_bytes;
    int32_t hash_algo;
    int32_t hash_iterations;
    int32_t session_key_encryption_key_bytes;
#define ECRYPTFS_PERSISTENT_PASSWORD             0x01
#define ECRYPTFS_SESSION_KEY_ENCRYPTION_KEY_SET  0x02
    uint32_t flags;
    /* Iterated-hash concatenation of salt and passphrase */
    uint8_t session_key_encryption_key[ECRYPTFS_MAX_KEY_BYTES];
    uint8_t signature[ECRYPTFS_PASSWORD_SIG_SIZE + 1];
    /* Always in expanded hex */
    uint8_t salt[ECRYPTFS_SALT_SIZE];
};

struct ecryptfs_auth_tok {
    uint16_t version; /* 8-bit major and 8-bit minor */
    uint16_t token_type;
#define ECRYPTFS_ENCRYPT_ONLY 0x00000001
    uint32_t flags;
    struct ecryptfs_session_key session_key;
    uint8_t reserved[32];
    union {
        struct ecryptfs_password password;
    } token;
}  __attribute__ ((packed));

int generate_passphrase_sig(
    char *passphrase_sig,
    char *fekek,
    char *salt,
    char *passphrase);

int generate_payload(
    struct ecryptfs_auth_tok *auth_tok,
    char *passphrase_sig,
    char *salt,
    char *session_key_encryption_key);
