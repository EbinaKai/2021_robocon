#include "mbed.h"
#include "SerialCtrl_OS6/SerialCtrl.h"
#include "OmniMove/OmniMove.h"  //インクルァぁドしてネ
#define sensitivity 0.1
#define WINGSPEED 0.3
#define ARMSPEED 0.3

//ピン宣言
DigitalOut Digital[5] =    {DigitalOut(PC_8),   //モーター用PIN
                            DigitalOut(PC_5),
                            DigitalOut(PA_11),
                            DigitalOut(PB_13),  //羽用
                            DigitalOut(PC_4)
                           };  //腕用
PwmOut Pwm[5] =            {PwmOut(PC_9),
                            PwmOut(PA_6),
                            PwmOut(PA_7),
                            PwmOut(PA_10),
                            PwmOut(PB_3)
                           };

void output(float value,int num);
void coordinate(int X,int Y,float* r,float* theta);

//グローバル変数を用意
float motor_output[4];
float r,theta,angle,X,Y,R,L,r_mv=0,current_X=0,current_Y=0;
int i,fail;
unsigned char can_data[8] = {0,0,0,0,0,0,0,0}; //CAN通信で受け取った情報を格納する変数

//Serial
BufferedSerial FEP(PC_10,PC_11,38400);  //シリアル通信のピン宣言
Timer checker;                          //タイマーの宣言
SerialCtrl wire(&FEP, &checker);        //シリアル通信のライブラリ宣言

OmniMove mekanamu; //TickerやTimerのように宣言

//モーター出力関数
void output(float value,int num){
    float pwm;
    int digital;
    if(value > 0) {
        digital = 0;
        pwm = value;
    } else {
        digital = 1;
        pwm = (value + 1);
    }
    Digital[num] = digital;
    Pwm[num] = pwm;
}

int main()
{
    //initialize
    checker.reset(); //timer reset
    checker.start(); //timer start
    wire.param(0xAF, 0xED); //header and footer

    mekanamu.setup(3,30);   //車輪の個数、位置を設定(1度宣言すればいい)

    //周期設定
    Pwm[0].period_us(83);
    Pwm[1].period_us(83);
    Pwm[2].period_us(83);
    Pwm[3].period_us(83);
    Pwm[4].period_us(83);

    mekanamu.input_polar(0,0,0);
    mekanamu.output(motor_output);
    for(i=0; i<3; i++) {
        output(motor_output[i],i);
    }

    //変数の初期化
    X = Y = R = L = angle = 0;
    int fail = 0,loop_cnt;
    printf("start\n");

    //main loop
    while(true) {
        if(wire.get()) {
            fail = 0;

            //座標計算
            X = (float)wire.data[LX];
            Y = (float)wire.data[LY];
            R = (float)wire.data[R2];
            L = (float)wire.data[L2];
            angle = R - L;
            Y = (Y - 255) * -1;
            X = X / 255;
            Y = Y / 255;
            angle = angle / 255;
            X = (X - 0.5) * -2;
            Y = (Y - 0.5) * -2;

            //初期位置誤差吸収
            if (X < sensitivity && X > sensitivity) X = 0;
            if (Y < sensitivity && Y > sensitivity) Y = 0;
            if (angle < sensitivity && angle > sensitivity) angle = 0;

            //台形制御
            if(abs(X-current_X)<0.11) {
                current_X = X;
            } else if(X>current_X) {
                current_X += 0.1;
            } else if(X<current_X) {
                current_X -= 0.1;
            }
            if(abs(Y-current_Y)<0.11) {
                current_Y = Y;
            } else if (Y>current_Y) {
                current_Y += 0.1;
            } else if(Y<current_Y) {
                current_Y -= 0.1;
            }

            //極座標変換
            r = sqrt(current_X*current_X + current_Y*current_Y);
            theta = atan2(current_Y,current_X); //atan2を使わないと分母が0になる場合がある
            //r = sqrt(X*X + Y*Y);
            //theta = atan2(Y,X);
            theta = theta * 180 / 3.141592;

            //モーター出力
            mekanamu.input_polar(r,theta,angle);
            mekanamu.output(motor_output);
            for(i=0; i<3; i++) {
                output(motor_output[i],i);
            }

            //羽オンオフ制御
            if(wire.data[B2] & R1) {
                if(wire.data[B1] & TRIANGLE) {
                    Digital[3] = 1;
                    Pwm[3] = 1 - ARMSPEED;
                    printf("arm close\n");
                } else {
                    Digital[3] = 0;
                    Pwm[3] = ARMSPEED;
                    printf("arm open\n");
                }
            } else {
                Digital[3] = 0;
                Pwm[3] = 0;
            }

            //羽オンオフ制御≈
            if(wire.data[B2] & L1) {
                if(wire.data[B1] & TRIANGLE) {
                    Digital[4] = 1;
                    Pwm[4] = 1 - WINGSPEED;
                    printf("wing close\n");
                } else {
                    Digital[4] = 0;
                    Pwm[4] = WINGSPEED;
                    printf("wing open\n");
                }
            } else if(r > 0.15){
                Digital[4] = 1;
                Pwm[4] = 0.8;
            } else {
                Digital[4] = 0;
                Pwm[4] = 0;
            }
        } else {
            fail++;
            printf("error\n");
            if(fail > 5) {
                //emergency stop
                mekanamu.input_polar(0,0,0);
                mekanamu.output(motor_output);
                for(i=0; i<3; i++) {
                    output(motor_output[i],i);
                }
            }
        }
    }
}