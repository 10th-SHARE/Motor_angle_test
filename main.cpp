#include "mbed.h"
#include "mext_encoder_stm32/Encoder.h"
#include "CalPID.h"
#include "NsPwmOut.h"
#include "MotorController.h"
#include <cstdio>

// デバックをしたいときはINTERACTIVEをdefine
#define INTERACTIVE

#define RESOLUTION 2048  // 分解能
#define DELTA_T 0.01    // 制御周期
#define DUTY_MAX 0.8     // duty出力上限
#define OMEGA_MAX  50    // 角速度の上限[rad/s]

constexpr auto time_step = 10ms;
constexpr auto time_step_secs = std::chrono::duration<float>(time_step).count();
const double deg_rad = 3.14 / 180.0;
const double rad_deg = 180.0 / 3.14;

float target_angle_rad = 0.0f;
float target_angle_deg = 0.0f;
float angle_kp = 8.0f;
float angle_kd = 0.1f;

CalPID speed_pid(0.1, 0.0, 0.0002, DELTA_T, DUTY_MAX);     //速度制御のPID
CalPID angle_omega_pid(8.0, 0.0, 0.1, DELTA_T, OMEGA_MAX);  //角度制御のPID

Ec ec(PB_5, PB_4, RESOLUTION);
MotorController motor(PA_3, PA_1, DELTA_T, ec, speed_pid, angle_omega_pid);
// Ec ec(PB_5, PB_4, RESOLUTION);
// MotorController motor(PA_1, PA_3, DELTA_T, ec, speed_pid, angle_omega_pid);

UnbufferedSerial ser(CONSOLE_TX, CONSOLE_RX);
InterruptIn photo(PA_7);

Ticker ticker;

void handle_key();
void control_start();
void control_loop();
void control_stop();
void wait_for_key(char k);
void move_to_zero();


int main() {

    ec.setGearRatio((double) 7/4);
    motor.setEquation(0.04611,0.023888,-0.04254,0.031945);

	printf("starting...\n");

    #ifdef INTERACTIVE // デバック時はスペースを押すと始まる
        printf("press space to start.\n");
        wait_for_key(' ');
        printf("\n");
    #else // そうでないときは1s後に始まる
        thread_sleep_for(1000);
    #endif

    // ゼロ点合わせ
	printf("finding zero...\n");
	//move_to_zero();

    printf("target_angle_deg: %f\n", target_angle_deg);
    printf("angle_kp: %f\n", angle_kp);
    printf("angle_kd: %f\n", angle_kd);
    printf("a: target_angle_deg -5[deg], d: target_angle_deg +5[deg]\n");
    printf("q: kp +0.5, e: kp -0.5\n");
    printf("z: kd*2, c: kd*0.5\n");
    printf("s: Ac_stop, w : Ac_start\n");

    #ifdef INTERACTIVE // デバック時はスペースを押すと始まる
        printf("press space to continue\n");

        wait_for_key(' ');
        printf("\nSTART\n");
    #endif

    thread_sleep_for(50);

    //CAN通信するならここにコード

    // シリアル割り込みが発生するたびに関数を呼び出す
    ser.attach(handle_key);

    while(1){
		printf("%f, target: %f, now: %f\n", ec.getOmega(), target_angle_deg, ec.getDeg());
		printf("%d\n", photo.read());
        thread_sleep_for(500);
    }
}

// 文字が入力されると呼び出される関数
// a: 目標角度 -5°、d: 目標角度 +5°
// q: kp +0.5、e: kp -0.5
// z: kd 2倍、c: kd 0.5倍
// s: 角度制御停止、w: 角度制御スタート
void handle_key() {
	if (ser.readable()) {
		char c;
		ser.read(&c, 1);
		if (c == 'a') {
			target_angle_deg -= 5.0f;
            target_angle_rad = target_angle_deg * deg_rad;
            printf("\ntarget_angle_deg: %f\n", target_angle_deg);
		} else if (c == 'd') {
			target_angle_deg += 5.0f;
            target_angle_rad = target_angle_deg * deg_rad;
            printf("\ntarget_angle_deg: %f\n", target_angle_deg);
		} else if (c == 'q') {
			angle_kp -= 0.5;
            angle_omega_pid.setParameter(angle_kp, 0.0, angle_kd);
            printf("\nangle_kp: %f, ", angle_kp);
            printf("angle_kd: %f\n", angle_kd);
		} else if (c == 'e') {
			angle_kp += 0.5;
            angle_omega_pid.setParameter(angle_kp, 0.0, angle_kd);
            printf("\nangle_kp: %f, ", angle_kp);
            printf("angle_kd: %f\n", angle_kd);
		} else if (c == 'z') {
			angle_kd *= 0.5;
            angle_omega_pid.setParameter(angle_kp, 0.0, angle_kd);
            printf("\nangle_kp: %f, ", angle_kp);
            printf("angle_kd: %f\n", angle_kd);
		} else if (c == 'c') {
			angle_kd *= 2.0;
            angle_omega_pid.setParameter(angle_kp, 0.0, angle_kd);
            printf("\nangle_kp: %f, ", angle_kp);
            printf("angle_kd: %f\n", angle_kd);
		} else if (c == 's') {
			control_stop();
            // printf("\nAc stop\n");
		} else if (c == 'w') {
			control_start();
            printf("\nAc start\n");
		} else if (c == 'r') {
			move_to_zero();
            printf("\ninitial zero\n");
		} else {
            printf("\n");
        }
	}
}

void control_start() {
    ticker.attach(control_loop, time_step);
    thread_sleep_for(time_step_secs);
}

void control_loop() {
    motor.Ac(target_angle_rad);
}

void control_stop() {
    ticker.detach();
    motor.stop();
}

void wait_for_key(char k) {
    while (true) {
        char c;
        ser.read(&c, 1);
        if (c == k) {
            break;
        }
    }
}

// ゼロ点合わせをする関数
void move_to_zero() {
    // 境目までduty0.06でゆっくり動かして止める
	if (photo.read() == 0) {
		while (photo.read() == 0) {
			motor.turn(-0.06f);
		}
	} else {
		while (photo.read() == 1) {
			motor.turn(0.06f);
		}
	}
    motor.stop();
    target_angle_deg = 0.0f;
    target_angle_rad = 0.0f;
    ec.reset();
}
