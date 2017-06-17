/**
 * Copyright (C) 2006 International Business Machines Corp.
 *   Author(s): Michael A. Halcrow <mahalcro@us.ibm.com>
 *        Tyler Hicks <tyhicks@ou.edu>
 * Copyright (C) 2013 Sony Mobile Communications AB.
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

#define LOG_TAG "ecryptfs"
#include "cutils/log.h"
#include <stdio.h>
#include "ecryptfs.h"
#include <string.h>
#include <errno.h>
#include <openssl/sha.h>

static int do_hash(char *src, int src_size, char *dst)
{
    SHA512_CTX ctx;
    if (src == NULL || dst == NULL) {
        return 1;
    }

    SHA512_Init(&ctx);
    SHA512_Update(&ctx, src, src_size);
    SHA512_Final((unsigned char*)dst, &ctx);
    return 0;
}

static inline void to_hex(char *dst, char *src, int src_size)
{
    int x;

    for (x = 0; x < src_size; x++)
        snprintf(&dst[x*2], 3, "%.2x", (unsigned char)src[x] );
    dst[src_size*2] = '\0';
}

/**
 * TODO: We need to support more hash algs
 * @fekek: ECRYPTFS_MAX_KEY_BYTES bytes of allocated memory
 *
 * @passphrase A NULL-terminated char array
 *
 * @salt A salt
 *
 * @passphrase_sig An allocated char array into which the generated
 * signature is written; PASSWORD_SIG_SIZE bytes should be allocated
 *
 */
int
generate_passphrase_sig(char *passphrase_sig, char *fekek,
                        char *salt, char *passphrase)
{
    char salt_and_passphrase[ECRYPTFS_MAX_PASSPHRASE_BYTES
                             + ECRYPTFS_SALT_SIZE];
    int passphrase_size;
    int dig_len = SHA512_DIGEST_LENGTH;
    char buf[SHA512_DIGEST_LENGTH];
    int hash_iterations = ECRYPTFS_DEFAULT_NUM_HASH_ITERATIONS;
    int rc = 0;

    passphrase_size = strnlen(passphrase, ECRYPTFS_MAX_PASSPHRASE_BYTES + 1);
    if (passphrase_size > ECRYPTFS_MAX_PASSPHRASE_BYTES) {
        passphrase_sig = NULL;
        SLOGE("Passphrase too large (%d bytes)", passphrase_size);
        return -EINVAL;
    }
    memcpy(salt_and_passphrase, salt, ECRYPTFS_SALT_SIZE);
    memcpy((salt_and_passphrase + ECRYPTFS_SALT_SIZE), passphrase,
           passphrase_size);
    if ((rc = do_hash(salt_and_passphrase,
                      (ECRYPTFS_SALT_SIZE + passphrase_size), buf))) {
        return rc;
    }
    hash_iterations--;
    while (hash_iterations--) {
        if ((rc = do_hash(buf, dig_len, buf))) {
            return rc;
        }
    }
    memcpy(fekek, buf, ECRYPTFS_MAX_KEY_BYTES);
    if ((rc = do_hash(buf, dig_len, buf))) {
        return rc;
    }
    to_hex(passphrase_sig, buf, ECRYPTFS_SIG_SIZE);
    return 0;
}

/**
 * Creates the authentication token used by the kernel to encrypt files.
 *
 * @auth_tok The authentication token sent to the kernel keyring
 *
 * @passphrase_sig A char array with the generated signature
 *
 * @salt A salt
 *
 * @session_key_encryption_key: ECRYPTFS_MAX_KEY_BYTES bytes of allocated memory
 *
 * @return Zero on success
 */
int
generate_payload(struct ecryptfs_auth_tok *auth_tok, char *passphrase_sig,
                 char *salt, char *session_key_encryption_key)
{
    int rc = 0;
    memset(auth_tok, 0, sizeof(struct ecryptfs_auth_tok));
    auth_tok->version = (((uint16_t)(VERSION_MAJOR << 8) & 0xFF00)
                         | ((uint16_t)VERSION_MINOR & 0x00FF));
    auth_tok->token_type = 0;
    strncpy((char *)auth_tok->token.password.signature, passphrase_sig,
            ECRYPTFS_PASSWORD_SIG_SIZE);
    memcpy(auth_tok->token.password.salt, salt, ECRYPTFS_SALT_SIZE);
    memcpy(auth_tok->token.password.session_key_encryption_key,
           session_key_encryption_key, ECRYPTFS_MAX_KEY_BYTES);
    /* TODO: Make the hash parameterizable via policy */
    auth_tok->token.password.session_key_encryption_key_bytes = 32;
    auth_tok->token.password.flags |=
        ECRYPTFS_SESSION_KEY_ENCRYPTION_KEY_SET;
    /* The kernel code will encrypt the session key. */
    auth_tok->session_key.encrypted_key[0] = 0;
    auth_tok->session_key.encrypted_key_size = 0;
    /* Default; subject to change by kernel eCryptfs */
    auth_tok->token.password.hash_algo = PGP_DIGEST_ALGO_SHA512;
    auth_tok->token.password.flags &= ~(ECRYPTFS_PERSISTENT_PASSWORD);
    return rc;
}
