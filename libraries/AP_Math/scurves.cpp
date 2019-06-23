/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_Math/AP_Math.h>
#include "scurves.h"

void scurves::Cal_T(float tin, float J0)
{
    enum jtype_t Jtype = JTYPE_CONSTANT;
    float J = J0;
    float T = oT[num_items] + tin;
    float A = oA[num_items] + J0*tin;
    float V = oV[num_items] + oA[num_items]*tin + 0.5*J0*sq(tin);
    float P = oP[num_items] + oV[num_items]*tin + 0.5*oA[num_items]*sq(tin) + (1/6)*J0*powf(tin,3);
    num_items++;

    oT[num_items] = T;
    oJtype[num_items] = Jtype;
    oJ[num_items] = J;
    oA[num_items] = A;
    oV[num_items] = V;
    oP[num_items] = P;
}

void scurves::Cal_JS1(float tj, float Jp)
{
    float Beta = pi/tj;
    float Alpha = Jp/2;
    float AT = Alpha*tj;
    float VT = Alpha*(sq(tj)/2 - 2/sq(Beta));
    float PT = Alpha*((-1/sq(Beta))*tj + (1/6)*powf(tj,3));

    enum jtype_t Jtype = JTYPE_POSITIVE;
    float J = Jp;
    float T = oT[num_items] + tj;
    float A = oA[num_items] + AT;
    float V = oV[num_items] + oA[num_items]*tj + VT;
    float P = oP[num_items] + oV[num_items]*tj + 0.5*oA[num_items]*sq(tj) + PT;
    num_items++;

    oT[num_items] = T;
    oJtype[num_items] = Jtype;
    oJ[num_items] = J;
    oA[num_items] = A;
    oV[num_items] = V;
    oP[num_items] = P;
}

void scurves::Cal_JS2(float tj, float Jp)
{
    float Beta = pi/tj;
    float Alpha = Jp/2;
    float AT = Alpha*tj;
    float VT = Alpha*(sq(tj)/2 - 2/sq(Beta));
    float PT = Alpha*((-1/sq(Beta))*tj + (1/6)*powf(tj,3));
    float A2T = Jp*tj;
    float V2T = Jp*sq(tj);
    float P2T = Alpha*((-1/sq(Beta))*2*tj +(4/3)*powf(tj,3));

    enum jtype_t Jtype = JTYPE_NEGATIVE;
    float J = Jp;
    float T = oT[num_items] + tj;
    float A = (oA[num_items]-AT) + A2T;
    float V = (oV[num_items]-VT) + (oA[num_items]-AT)*tj + V2T;
    float P = (oP[num_items]-PT) + (oV[num_items]-VT)*tj + 0.5*(oA[num_items]-AT)*sq(tj) + P2T;
    num_items++;

    oT[num_items] = T;
    oJtype[num_items] = Jtype;
    oJ[num_items] = J;
    oA[num_items] = A;
    oV[num_items] = V;
    oP[num_items] = P;
}

void scurves::Cal_tj_Jp_Tcj(float tj, float Jp, float Tcj)
{
    Cal_JS1(tj, Jp);
    Cal_T(Tcj, Jp);
    Cal_JS2(tj, Jp);
}

void scurves::Cal_Pn(float Pp)
{
    float tj = otj;
    float Jp = oJp;
    float Ap = oAp;
    float Vp = oVp;
    float A0 = oA(num_items);
    float V0 = oV(num_items);
    float P0 = oP(num_items);

    float Jp, t2, t4, t6;
    Cal_Pos(tj, V0, P0, Jp, Ap, Vp, Pp, Jp, t2, t4, t6);

    Cal_tj_Jp_Tcj(tj, Jp, t2);
    Cal_T(t4, 0.0);
    Cal_tj_Jp_Tcj(tj, -Jp, t6);
    float Tcv = (Pp-oP(num_items))/oV(num_items);
    Cal_T(Tcv, 0.0);

    Cal_Pos(tj, 0, P0, Jp, Ap, Vp, Pp*2, Jp, t2, t4, t6);
    Cal_T(Tcv, 0.0);
    Cal_tj_Jp_Tcj(tj, -Jp, t6);
    Cal_T(t4, 0.0);
    Cal_tj_Jp_Tcj(tj, Jp, t2);
}

void scurves::Cal_Pos(float tj, float V0, float P0, float Jp, float Ap, float Vp, float Pp,
                      float& Jp_out, float& t2_out, float& t4_out, float& t6_out)
{
    Ap = MIN(MIN(Ap, (Vp - V0)/(2*tj)), (Pp - P0 + 4*V0*tj)/(4*sq(tj)));
    if (fabsf(Ap) < Jp*tj) {
        Jp_out = Ap/tj;
        t2_out = 0;
        if ((Vp <= V0 + 2*Ap*tj) || (Pp <= P0 + 4*V0*tj + 4*Ap*sq(tj))) {
            t4_out = 0;
        } else {
            t4_out = MIN(-(sq(Ap)/Jp + tj*Ap + V0 - Vp)/Ap, \
                MAX(-(3*sq(Ap) - 2*sqrtf(powf(Ap,4)/4 + sq(Jp)*sq(V0) + (sq(Ap)*sq(Jp)*sq(tj))/4 - 2*Ap*sq(Jp)*P0 + 2*Ap*sq(Jp)*Pp - sq(Ap)*Jp*V0 + (powf(Ap,3)*Jp*tj)/2 - Ap*sq(Jp)*V0*tj) + 2*Jp*V0 + 3*Ap*Jp*tj)/(2*Ap*Jp), \
                -(3*sq(Ap) + 2*sqrtf(powf(Ap,4)/4 + sq(Jp)*sq(V0) + (sq(Ap)*sq(Jp)*sq(tj))/4 - 2*Ap*sq(Jp)*P0 + 2*Ap*sq(Jp)*Pp - sq(Ap)*Jp*V0 + (powf(Ap,3)*Jp*tj)/2 - Ap*sq(Jp)*V0*tj) + 2*Jp*V0 + 3*Ap*Jp*tj)/(2*Ap*Jp)) \
                );
        }
    } else {
        if ((Vp < sq(Ap)/Jp + tj*Ap + V0) || (Pp < P0 + (powf(Ap,3) + Ap*Jp*(2*V0 + 2*Ap*tj))/sq(Jp) + 2*V0*tj + Ap*sq(tj))) {
            Ap = MIN(MIN(Ap, \
                     MAX(-(Jp*(tj + sqrtf((Jp*sq(tj) - 4*V0 + 4*Vp)/Jp)))/2, \
                     -(Jp*(tj - sqrtf((Jp*sq(tj) - 4*V0 + 4*Vp)/Jp)))/2)), \
                     ((sq(Jp)*sq(tj))/9 - (2*V0*Jp)/3)/(sqrtf(sq((sq(Jp)*P0)/2 - (sq(Jp)*Pp)/2 + (8*powf(Jp,3)*powf(tj,3))/27 - (Jp*tj*(sq(Jp)*sq(tj) + 2*V0*Jp))/3 + sq(Jp)*V0*tj) - powf(((sq(Jp)*sq(tj))/9 - (2*Jp*V0)/3),3)) - (sq(Jp)*P0)/2 + (sq(Jp)*Pp)/2 - (8*powf(Jp,3)*powf(tj,3))/27 + (Jp*tj*(sq(Jp)*sq(tj) + 2*V0*Jp))/3 - sq(Jp)*V0*tj)^(1/3) - (2*Jp*tj)/3 + ((sq((sq(Jp)*P0)/2 - (sq(Jp)*Pp)/2 + (8*powf(Jp,3)*powf(tj,3))/27 - (Jp*tj*(sq(Jp)*sq(tj) + 2*V0*Jp))/3 + sq(Jp)*V0*tj) - powf(((sq(Jp)*sq(tj))/9 - (2*V0*Jp)/3),3))^(1/2) - (sq(Jp)*P0)/2 + (sq(Jp)*Pp)/2 - (8*powf(Jp,3)*powf(tj,3))/27 + (Jp*tj*(sq(Jp)*sq(tj) + 2*V0*Jp))/3 - sq(Jp)*V0*tj)^(1/3));
            t4_out = 0;
        } else {
            t4_out = MIN(-(sq(Ap)/Jp + tj*Ap + V0 - Vp)/Ap, \
                         MAX(-(3*sq(Ap) - 2*(powf(Ap,4)/4 + sq(Jp)*sq(V0) + (sq(Ap)*sq(Jp)*sq(tj))/4 - 2*Ap*sq(Jp)*P0 + 2*Ap*sq(Jp)*Pp - sq(Ap)*Jp*V0 + (powf(Ap,3)*Jp*tj)/2 - Ap*sq(Jp)*V0*tj)^(1/2) + 2*Jp*V0 + 3*Ap*Jp*tj)/(2*Ap*Jp), \
                         -(3*sq(Ap) + 2*(powf(Ap,4)/4 + sq(Jp)*sq(V0) + (sq(Ap)*sq(Jp)*sq(tj))/4 - 2*Ap*sq(Jp)*P0 + 2*Ap*sq(Jp)*Pp - sq(Ap)*Jp*V0 + (powf(Ap,3)*Jp*tj)/2 - Ap*sq(Jp)*V0*tj)^(1/2) + 2*Jp*V0 + 3*Ap*Jp*tj)/(2*Ap*Jp)) );
        }
        t2_out = Ap/Jp - tj;
    }
    t6_out = t2_out;
}

void scurves::runme(float t, float& Jt_out, float& At_out, float& Vt_out, float& Pt_out)
{
    pnt = find(t<obj.oT, 1, 'first');
    if (pnt == 1) {
        T = obj.oT(pnt);
        Jtype = 0;
        Jp = 0.0;
        A0 = obj.oA(pnt);
        V0 = obj.oV(pnt);
        P0 = obj.oP(pnt);
    } else if (isempty(pnt)) {
        T = obj.oT(end);
        Jtype = 0;
        Jp = 0.0;
        A0 = obj.oA(end);
        V0 = obj.oV(end);
        P0 = obj.oP(end);
    } else {
        Jtype = obj.oJtype(pnt);
        Jp = obj.oJ(pnt);
        T = obj.oT(pnt-1);
        A0 = obj.oA(pnt-1);
        V0 = obj.oV(pnt-1);
        P0 = obj.oP(pnt-1);
    }

    switch (Jtype) {
        case JTYPE_POSITIVE:
            [Jt, At, Vt, Pt] = JSegment1(obj, t-T, Jp, A0, V0, P0);
            break;
        case JTYPE_NEGATIVE:
            [Jt, At, Vt, Pt] = JSegment2(obj, t-T, Jp, A0, V0, P0);
            break;
        default:
            [Jt, At, Vt, Pt] = JConst(obj, t-T, Jp, A0, V0, P0);
            break;
    }
}

void scurves::JConst(float t, float J0, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt)
{
    Jt = J0;
    At = A0 + J0*t;
    Vt = V0 + A0*t + 0.5*J0*t^2;
    Pt = P0 + V0*t + 0.5*A0*t^2 + (1/6)*J0*t^3;
}

void scurves::JSegment1(float t, float Jp, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt)
{
    tj = obj.otj;
    Alpha = Jp/2;
    Beta = pi/tj;
    Jt = Alpha*(1 - cos(Beta*t));
    At = A0 + Alpha*t - (Alpha/Beta)*sin(Beta*t);
    Vt = V0 + A0*t + (Alpha/2)*t^2 + (Alpha/Beta^2)*cos(Beta*t) - Alpha/Beta^2;
    Pt = P0 + V0*t + 0.5*A0*t^2 + (-Alpha/Beta^2)*t + Alpha*t^3/6 + (Alpha/Beta^3)*sin(Beta*t);
}

void scurves::JSegment2(float t, float Jp, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt)
{
    tj = obj.otj;
    Alpha = Jp/2;
    Beta = pi/tj;
    AT = Alpha*tj;
    VT = Alpha*(tj^2/2 - 2/Beta^2);
    PT = Alpha*((-1/Beta^2)*tj + (1/6)*tj^3);
    Jt = Alpha*(1 - cos(Beta * (t+tj)));
    At = (A0-AT) + Alpha*(t+tj) - (Alpha / Beta) * sin(Beta * (t+tj));
    Vt = (V0-VT) + (A0-AT)*t + 0.5*Alpha*(t+tj)^2 + (Alpha/Beta^2)*cos(Beta*(t+tj)) - Alpha/Beta^2;
    Pt = (P0-PT) + (V0-VT)*t + 0.5*(A0-AT)*t^2 + (-Alpha/Beta^2)*(t+tj) + (Alpha/6)*(t+tj)^3 + (Alpha/Beta^3)*sin(Beta*(t+tj));
}
