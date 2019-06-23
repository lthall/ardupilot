#pragma once

#include <AP_Common/AP_Common.h>

class scurves {
public:

    // constructor
    scurves(float tj, float Jp, float Ap, float Vp) :
        otj(tj), oJp(Jp), oAp(Ap), oVp(Vp)
    {}

    void Cal_T(float tin, float J0);
    void Cal_JS1(float tj, float Jp);
    void Cal_JS2(float tj, float Jp);
    void Cal_tj_Jp_Tcj(float tj, float Jp, float Tcj);

    void Cal_Pn(float Pp);
    void Cal_Pos(float tj, float V0, float P0, float Jp, float Ap, float Vp, float Pp,
                 float& Jp_out, float& t2_out, float& t4_out, float& t6_out);

    void runme(float t, float& Jt_out, float& At_out, float& Vt_out, float& Pt_out);
    void JConst(float t, float J0, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt);
    void JSegment1(float t, float Jp, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt)
    void JSegment2(float t, float Jp, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt);

private:

    const static uint16_t array_size_max = 16;

    // scurve segment types
    enum jtype_t {
        JTYPE_CONSTANT,
        JTYPE_POSITIVE,
        JTYPE_NEGATIVE
    };

    // members
    float otj;
    float oJp;
    float oAp;
    float oVp;

    // arrays
    uint16_t num_items;
    float oJ[array_size_max];
    enum jtype_t oJtype[array_size_max];
    float oT[array_size_max];
    float oA[array_size_max];
    float oV[array_size_max];
    float oP[array_size_max];
};
