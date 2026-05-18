// 气动状态枚举（共用）
#ifndef PNEUMATIC_STATE_H
#define PNEUMATIC_STATE_H

enum PneumaticState {
    PNEUMATIC_IDLE,      // 空闲
    PNEUMATIC_INFLATING, // 充气中
    PNEUMATIC_HOLDING,   // 维持气压
    PNEUMATIC_DEFLATING // 放气中
};

#endif