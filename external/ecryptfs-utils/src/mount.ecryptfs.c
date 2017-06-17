/* mount.ecryptfs.c
 *
 * Copyright (C) 2013 Sony Mobile Communications Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#define LOG_TAG "mount.ecryptfs"
#include "cutils/log.h"
#include <stdio.h>
#include <sys/mount.h>
#include <unistd.h>

#include "ecryptfs.h"
#include "keyhandling.h"

#define MOUNT_OPTIONS_LENGTH 512

static void usage()
{
    SLOGE("usage: mount.ecryptfs -f <folder> (-d <destination> | -p <passphrase> | -s <signature>"
            " | -e <extension>)\n"
            " -f    the folder to mount\n"
            " -d    target folder\n"
            " -p    passphrase for the ecryptfs mount\n"
            " -s    signature of a key loaded in kernel keyring\n"
            " -e    file extension to be filtered");
}

int main(int argc, char *argv[])
{
    long ret = 0;
    char *folder = NULL;
    char *dest_folder = NULL;
    char *signature = NULL;
    char *passphrase = NULL;
    char *extension = NULL;
    char options[MOUNT_OPTIONS_LENGTH] = {0};
    struct ecryptfs_auth_tok auth_tok;
    char auth_tok_sig[ECRYPTFS_SIG_SIZE_HEX + 1] = {0};
    char fekek[ECRYPTFS_MAX_KEY_BYTES] = {0};
    char salt[ECRYPTFS_SALT_SIZE + 1] = "whatever";
    int c;

    /* check parameters */
    while ((c = getopt(argc, argv, "f:d:p:s:e:")) != -1) {
        switch (c) {
        case 'f':
            folder = optarg;
            break;
        case 'd':
            dest_folder = optarg;
            break;
        case 'p':
            passphrase = optarg;
            break;
        case 's':
            signature = optarg;
            break;
        case 'e':
            extension = optarg;
            break;
        default:
            SLOGE("\n");
            usage();
            ret = 1;
            goto out;
        }
    }

    if (folder == NULL) {
        SLOGE("folder path has to be given");
        ret = 2;
        goto out;
    } else if (folder[strnlen(folder, MOUNT_OPTIONS_LENGTH) - 1] == '/') {
        folder[strnlen(folder, MOUNT_OPTIONS_LENGTH) - 1] = 0;
    }

    if (dest_folder != NULL &&
           dest_folder[strnlen(dest_folder, MOUNT_OPTIONS_LENGTH) - 1] == '/') {
        dest_folder[strnlen(dest_folder, MOUNT_OPTIONS_LENGTH) - 1] = 0;
    }

    if ((passphrase == NULL && signature == NULL) ||
            (passphrase != NULL && signature != NULL)) {
        SLOGE("either passphrase or signature has to be used");
        ret = 3;
        goto out;
    }

    if (passphrase != NULL) {
        /* generate key and mount key payload */
        ret = generate_passphrase_sig(auth_tok_sig, fekek, salt, passphrase);
        if (ret != 0) {
            SLOGE("failed to generate pass phrase");
            ret = 4;
            goto out;
        }

        generate_payload(&auth_tok, auth_tok_sig, salt, fekek);
    } else {
        if (strnlen(signature, MOUNT_OPTIONS_LENGTH) != sizeof(auth_tok_sig) - 1) {
            SLOGE("wrong format on signature");
            ret = 5;
            goto out;
        }

        strncpy(auth_tok_sig, signature, sizeof(auth_tok_sig));
    }

    /* link key */
    ret = keyctl_link(KEY_SPEC_USER_KEYRING, KEY_SPEC_SESSION_KEYRING);
    if (ret == -1) {
        SLOGE("failed to link key in keyring");
        ret = 6;
        goto out;
    }

    /* find key */
    ret = keyctl_search(KEY_SPEC_USER_KEYRING, "user", auth_tok_sig, 0);
    if (ret == -1) {
        if (passphrase != NULL) {
            /* add key to kernel keyring */
            ret = add_key("user", auth_tok_sig, (char*)&auth_tok, sizeof(auth_tok),
                          KEY_SPEC_USER_KEYRING);
            if (ret == -1) {
                SLOGE("failed to add key in keyring");
                ret = 7;
                goto out;
            }
        } else {
            SLOGE("could not find key (%s) in keyring", signature);
            ret = 8;
            goto out;
        }
    }

    /* create and call mount syscall */
    // Kitakami should keep the file name not encrypted so never pass the ecryptfs_fnek_sig option
    if (extension != NULL)
        snprintf(options, MOUNT_OPTIONS_LENGTH,
                 "ecryptfs_passthrough,ecryptfs_unlink_sigs,ecryptfs_sig=%s,"
                 "ecryptfs_key_bytes=32,ecryptfs_cipher=aes,ecryptfs_enable_filtering=%s",
                 auth_tok_sig, extension);
    else
        snprintf(options, MOUNT_OPTIONS_LENGTH,
                 "ecryptfs_passthrough,ecryptfs_unlink_sigs,ecryptfs_sig=%s,"
                 "ecryptfs_key_bytes=32,ecryptfs_cipher=aes",
                 auth_tok_sig);
    if (dest_folder != NULL) {
        ret = mount(folder, dest_folder, "ecryptfs", 0, options);
    } else {
        ret = mount(folder, folder, "ecryptfs", 0, options);
    }
    if (ret != 0) {
        SLOGE("failed to mount folder %s", folder);
        ret = 9;
        goto out;
    }

out:
    /* clear secret memory */
    memset(&auth_tok, 0, sizeof(auth_tok));
    memset(&fekek, 0, sizeof(fekek));
    if (passphrase != NULL) {
        memset(passphrase, 0, strnlen(passphrase, MOUNT_OPTIONS_LENGTH));
    }

    if (ret == 0) {
        SLOGD("%s", auth_tok_sig);
        printf("%s\n", auth_tok_sig);
    }

    return ret;
}
