
#include "cpr.h"
#include "cpr_int.h"

/* ======================================================== */
/* ================== Encoding ============================ */
/* ======================================================== */

struct message encode(int_type awb_lat, int_type awb_lon) 
{
    int_type nz = max(nl_awb(rlat_int(I, awb_lat)) - I, 1);
    struct message result = {I, encoding(60 - I, awb_lat), encoding(nz, awb_lon)};
    return result;
}
/* ======================================================== */
/* ==================== Local Decoding ==================== */
/* ======================================================== */

struct recovered_position local_dec(int_type reference_lat, int_type reference_longitude, struct message msg) 
{
    int_type r_lat = local_decode(60 - I, reference_lat, msg.yz);
    int_type r_lon = local_decode(max(nl_awb(r_lat) - I, 1), reference_longitude, msg.xz);
    struct recovered_position result = {1, r_lat, r_lon};
    return result;
}

// ���������� ������������� ����������� � ���������� � ����������.
