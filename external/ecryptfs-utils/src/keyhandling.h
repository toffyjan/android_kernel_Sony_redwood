/* keyhandling.h
 *
 * Copyright (C) 2013 Sony Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <sys/syscall.h>

#define KEYCTL_LINK 8
#define KEYCTL_UNLINK 9
#define KEYCTL_SEARCH 10
#define KEY_SPEC_SESSION_KEYRING -3
#define KEY_SPEC_USER_KEYRING -4 /* 0xfffffffc */

typedef int32_t key_serial_t;

/*
 * Wrapper for keyctl(KEYCTL_SEARCH, ...) syscall
 *
 * Search for a key in a keyring.
 */
inline long keyctl_search(
    key_serial_t ringid,
    const char *type,
    const char *description,
    key_serial_t destringid)
{
    return syscall(__NR_keyctl, KEYCTL_SEARCH, ringid, type, description, destringid);
}

/*
 * Wrapper for keyctl(KEYCTL_LINK, ...) syscall
 *
 * Link a key into a keyring.
 */
inline long keyctl_link(key_serial_t id, key_serial_t ringid)
{
    return syscall(__NR_keyctl, KEYCTL_LINK, id, ringid);
}

/*
 * Wrapper for keyctl(KEYCTL_UNLINK, ...) syscall
 *
 * Unlink a key from a keyring.
 */
inline long keyctl_unlink(key_serial_t id, key_serial_t ringid)
{
    return syscall(__NR_keyctl, KEYCTL_UNLINK, id, ringid);
}

/*
 * Wrapper for add_key syscall
 *
 * Add a key to the kernel's key management facility.
 */
inline key_serial_t add_key(
    const char *type,
    const char *description,
    const void *payload,
    size_t plen,
    key_serial_t keyring)
{
    return syscall(__NR_add_key, type, description, payload, plen, keyring);
}
