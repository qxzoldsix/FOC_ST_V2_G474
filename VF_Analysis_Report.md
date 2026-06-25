# VF 控制代码分析报告

> 项目: FOC_ST_V2_0601 | MCU: STM32G474 | 日期: 2026-06-03

---

## 调用链路

```
TIM1_UP (PWM 周期中断)
  └─ Foc_Control()                    [motorControl.c:9]
       └─ case 4: VF_Control_Run()    [VF.c:6]
            ├─ Grad_XieLv()           [Common_Math.c:1057]  频率斜坡
            ├─ SinCos_Table()         [Common_Math.c:1032]  正弦查表
            ├─ IPARK_Cale()           [Foc.c:23]            反 Park 变换
            └─ svpwm_Cale()           [svpwm.c:2]           SVPWM 计算
```

✅ `Foc_Control` 已在 `TIM1_UP_TIM16_IRQHandler` (stm32g4xx_it.c:331) 中被调用，非死代码。

---

## Bug 1 🔴 致命：除零错误

**文件:** `Foc_Control/src/VF.c:17`

```c
// 第 12 行只保护了 CurrentHz<=0 和 TargetVolt<=0.2f
if (pCtrl->CurrentHz <= 0.0f || pCtrl->TargetVolt <= 0.2f) {
    ...
} else {
    // 第 17 行：当 TargetHz=0 但 CurrentHz>0 时 → 除零！
    pCtrl->V_amp = VF_VOLTAGE_MIN +
        (pCtrl->TargetVolt - VF_VOLTAGE_MIN) * (pCtrl->CurrentHz / pCtrl->TargetHz);
}
```

**触发场景:** 用户设置 `TargetHz = 0` 停机，但 `Grad_XieLv` 仍在斜坡衰减中，`CurrentHz > 0` 且 `TargetHz == 0` → `CurrentHz / 0.0f` 产生 `inf` (IEEE 754)，后续 SVPWM 输出不可预测。

**修复:**
```c
if (pCtrl->CurrentHz <= 0.0f || pCtrl->TargetHz <= 0.0f || pCtrl->TargetVolt <= VF_VOLTAGE_MIN) {
```

---

## Bug 2 🔴 致命：设定值被意外清零

**文件:** `Foc_Control/src/VF.c:14`

```c
if (pCtrl->CurrentHz <= 0.0f || pCtrl->TargetVolt <= 0.2f) {
    pCtrl->V_amp = 0.0f;
    pCtrl->TargetVolt = 0.0f;   // ← BUG: 清零了用户的电压设定值
}
```

**问题:** 停机条件触发时，`TargetVolt`（用户通过 CAN/按键设定的目标电压）被永久清零。下次启动时如果外部未重新赋值 `TargetVolt`，条件 `TargetVolt <= 0.2f` 永远为真，电机永远无法启动。

**修复:** 删除 `pCtrl->TargetVolt = 0.0f;`。停机时只需将输出电压 `V_amp` 清零，不应修改用户设定值。

---

## Bug 3 🔴 致命：PWM 未输出到硬件

**文件:** `Foc_Control/src/VF.c:36-37`

```c
svpwm_Cale(&Svpwm_dq);
// ← 缺少 PWM_Update_From_SVPWM() 调用！
```

`svpwm_Cale` 只计算了 `SVPTa/SVPTb/SVPTc`，但从未调用 `PWM_Update_From_SVPWM()` 或 `PWM_SetDuty()` 将这些值写入 TIM1 的 CCR 比较寄存器。**电机绕组不会收到任何 PWM 驱动信号。**

**修复:** 在 `svpwm_Cale(&Svpwm_dq);` 之后加上：
```c
PWM_Update_From_SVPWM();
```

同理，`Foc_Control()` 的 case 2 (磁定向 FOC) 和 case 3 (有感 FOC) 也需要加上此调用。

---

## 问题 4 🟡 中等：魔数替代宏

**文件:** `Foc_Control/src/VF.c:12`

```c
if (pCtrl->CurrentHz <= 0.0f || pCtrl->TargetVolt <= 0.2f) {
```

`0.2f` 是 `VF_VOLTAGE_MIN` 的值，应直接使用宏：
```c
if (pCtrl->CurrentHz <= 0.0f || pCtrl->TargetVolt <= VF_VOLTAGE_MIN) {
```

---

## 问题 5 🟡 低：角度溢出用 if 而非 while

**文件:** `Foc_Control/src/VF.c:23-24`

```c
if (pCtrl->OpenTheta >= 2.0f * PI) pCtrl->OpenTheta -= 2.0f * PI;
```

当前参数下安全（单步最大 ~1.26 rad < 2π），但如果未来调大 `VF_FREQ_MAX` 或 `VF_TS`，单步可能超过 2π，用 `while` 更健壮：
```c
while (pCtrl->OpenTheta >= 2.0f * PI) pCtrl->OpenTheta -= 2.0f * PI;
while (pCtrl->OpenTheta < 0.0f)       pCtrl->OpenTheta += 2.0f * PI;
```

---

## 问题 6 🟡 低：ParkSinCos 缺少 static

**文件:** `Foc_Control/src/VF.c:3`

```c
Ang_SinCos ParkSinCos = Ang_SinCos_DEFAULTS;
```

该变量仅在 VF.c 内使用，应加 `static` 防止与其他模块的潜在符号冲突：
```c
static Ang_SinCos ParkSinCos = Ang_SinCos_DEFAULTS;
```

> **说明:** `ParkSinCos` 的类型 `Ang_SinCos` 和操作函数 `SinCos_Table()` 定义在 `Common_Math.h/c` 中，这是正确的。`ParkSinCos` 是 VF 模块的**工作实例变量**，放在 `VF.c` 中合理，只需加 `static` 修饰。

---

## 问题 7 🟡 低：编译器警告 — double 隐式转换

构建日志记录:
```
VF.c(23): warning: #1035-D: single-precision operand implicitly converted to double-precision
```

`2*PI` 中的 `2` 和 `65536.0f / (2*PI)` 中的 `65536.0f`，在 ARM CC 编译器中某些写法可能触发 double 提升。确保所有浮点字面量带 `f` 后缀。当前大部分已正确，警告可能来自 `(65536.0f / (2*PI))` 中 `(2*PI)` 的整数 `2` 参与运算。

**修复:**
```c
ParkSinCos.table_Angle = (uint16_t)(pCtrl->OpenTheta * (65536.0f / (2.0f * PI)));
```

---

## 总结

| # | 严重度 | 问题 | 位置 | 影响 |
|---|--------|------|------|------|
| 1 | 🔴 致命 | 除零：TargetHz=0 时无保护 | VF.c:17 | 输出 inf，电机失控 |
| 2 | 🔴 致命 | 设定值被意外清零 | VF.c:14 | 停机后无法再次启动 |
| 3 | 🔴 致命 | PWM 未写入硬件 CCR | VF.c:36 之后 | 电机无驱动信号 |
| 4 | 🟡 中等 | 魔数替代宏 | VF.c:12 | 可维护性 |
| 5 | 🟡 低 | 角度溢出用 if 而非 while | VF.c:23-24 | 参数变化后可能溢出 |
| 6 | 🟡 低 | ParkSinCos 缺 static | VF.c:3 | 潜在符号冲突 |
| 7 | 🟡 低 | double 隐式转换警告 | VF.c:25 | 无硬件 FP64 时影响性能 |

**最紧急的三个致命 Bug (1, 2, 3) 修复后，VF 控制应该可以正常工作。**



**已经排除以上问题，主要是查表格式不对应该是4096 而不是65536 越位了**