//
// Created by Jiatian Sun on 8/31/18.
//

#pragma once

#if !defined(__MITSUBA_BIDIR_PROBE_H_)
#define __MITSUBA_BIDIR_PROBE_H_

MTS_NAMESPACE_BEGIN

struct Probe{
public:
    enum EProbeType{
        ENORMAL = 0X00,
        EIDENTITY = 0X01,
        EROW = 0X02,
        ECOLUMN = 0X04,
        EDISPARITY = 0X08
    };
};

MTS_NAMESPACE_END

#endif //__MITSUBA_BIDIR_PROBE_H_
