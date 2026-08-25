/* Synthetic mfc_read feature module for the host/device integration contract.
 * It deliberately does no RF I/O: the resident RFID driver still exercises
 * request -> upload -> in-place load -> output -> unload, while this fixture
 * makes the 32-key/64-block records deterministic enough to validate the host
 * parser without requiring a particular card or antenna placement. */
#include "app_api.h"

#define MFC_REQ "/ramfs/.mfcrreq"
#define MFC_KEY "/ramfs/.mfckey"

int app_main(const fantasi_api_t *api)
{
#ifdef MOCK_MFC_BLOCK
    api->print("reading: mock MIFARE operation is blocked\r\n");
    for (;;) api->delay(10);
#else
    api->print("mfc: uid=DEADBEEF sak=08 atqa=0004\r\n");
    api->print("mfc: prng=weak\r\n");

    /* Mirror the real module's single-block wire contract when the resident
     * driver staged a normalized `BLOCK [KEY]` request. */
    int reqn = api->file_size(MFC_REQ);
    if (reqn > 0 && reqn < 32) {
        char req[32];
        if (api->pread(MFC_REQ, 0, req, (uint32_t)reqn) == reqn) {
            req[reqn] = 0;
            if (!(reqn >= 3 && req[0] == 'a' && req[1] == 'l' && req[2] == 'l')) {
                int p = 0, block = 0;
                while (req[p] >= '0' && req[p] <= '9') block = block * 10 + req[p++] - '0';
                const char *key = req[p] == ' ' ? req + p + 1 : "FFFFFFFFFFFF";
                api->printf("mfc: sec %02d keyA=%.12s\r\n", block / 4, key);
                api->printf("mfc: blk %02d = 00000000000000000000000000000000\r\n", block);
                api->print("reading: complete\r\n");
                api->printf("mfc: done block %d\r\n", block);
                api->remove(MFC_REQ);
                return 1;
            }
        }
        api->remove(MFC_REQ);
    }

    char preferred[13] = "FFFFFFFFFFFF";
    int keyn = api->file_size(MFC_KEY);
    if (keyn == 12 && api->pread(MFC_KEY, 0, preferred, 12) == 12) preferred[12] = 0;
    api->remove(MFC_KEY);

    for (int sector = 0; sector < 16; sector++) {
        api->printf("mfc: sec %02d keyA=%s\r\n", sector, preferred);
#ifdef MOCK_MFC_MALFORMED
        if (sector == 15)
            api->print("mfc: sec 15 keyB=FFFFFFFFFFF\r\n"); /* 11 digits */
        else
#endif
            api->printf("mfc: sec %02d keyB=A0A1A2A3A4A5\r\n", sector);

        for (int block = 0; block < 4; block++) {
            int absolute = sector * 4 + block;
#ifdef MOCK_MFC_MALFORMED
            if (absolute == 63)
                api->print("mfc: blk 63 = 0000000000000000000000000000000\r\n"); /* 31 digits */
            else
#endif
                api->printf("mfc: blk %02d = 00000000000000000000000000000000\r\n",
                            absolute);
        }
    }
    api->print("reading: complete\r\n");
    api->print("mfc: done 16 sectors\r\n");
    return 1;
#endif
}
