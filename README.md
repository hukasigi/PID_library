# PID_library
```
unsigned long now = micros();

if (now - last < CONTROL_CYCLE) return;
double dt = (now - last) * 1.e-6;
last      = now;
```

## RawPID
基本的なPIDです。
```
Pid hoge(kp,ki,kd);
```
で値を設定して呼び出します。
```
setTarget(double target);
```
で目標値を指定し、
```
update(double now_value,double dt);
```
を呼ぶことで差分を取りその差分に応じた出力が返ってきます。

