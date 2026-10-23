#include "arm.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 舵机协议/插补回归测试（沿用原 STM32_Arm6 的测试，行为未改）。
 * 不参与 Keil 固件编译；运行：python Tools/test_host.py */

static char replies[65536];
static size_t used;
static uint16_t pulses[ARM_SERVO_COUNT];
static bool enabled;
static unsigned enable_calls;
void arm_port_pwm(unsigned channel, uint16_t pulse) {
    assert(channel<ARM_SERVO_COUNT);assert(pulse>=1000 && pulse<=2000);pulses[channel]=pulse;
}
void arm_port_enable(bool on) { enabled=on;++enable_calls; }
void arm_port_reply(const char *text) {
    size_t n=strlen(text);
    if(used+n+1>=sizeof(replies))used=0;
    memcpy(replies+used,text,n+1);used+=n;
}
static void clean_log(void){used=0;replies[0]=0;}
static void send(arm_t *a,const char *s,uint32_t now){while(*s)arm_receive(a,(uint8_t)*s++,now);}
static void has(const char *text){assert(strstr(replies,text)!=NULL);}
static void tick_motion(arm_t *a,uint32_t start,unsigned steps){
    for(unsigned n=1;n<=steps;n++){
        uint32_t now=start+n*20;
        if(n%25==0)send(a,"$PING#",now);
        arm_tick(a,now);
    }
}

int main(void){
    arm_t a;assert(arm_init(&a,0));assert(!enabled);has("$READY,ARM6,OFF#");
    for(unsigned k=0;k<6;k++)assert(pulses[k]==1500);
    send(&a,"$90,90,90,90,90,90#",0);has("$ERR,DISABLED#");
    send(&a,"$PING#",1);has("$PONG#");
    send(&a,"$ON#",2);assert(enabled);assert(!a.link_lost);
    unsigned previous_enables=enable_calls;send(&a,"$ON#",3);assert(enable_calls==previous_enables);
    clean_log();
    send(&a,"noise$100,80,95",4);assert(!a.moving);
    send(&a,",90.5,90,85#\r\n",5);assert(a.moving);has("$ACK,MOVE#");
    tick_motion(&a,5,10);
    const uint16_t goal[]={1000,800,950,905,900,850};
    assert(!a.moving);for(unsigned k=0;k<6;k++)assert(a.current[k]==goal[k]);has("$DONE#");
    puts("PASS: boot OFF, ON idempotence, fragmented six-angle command, synchronized interpolation");

    const char *bad[]={"$90,90,90,90,90#","$90,90,90,90,90,90,90#","$90,90,90,90,90,181#", "$90,90,90,90,90,-1#", "$NaN,90,90,90,90,90#", "$90.12,90,90,90,90,90#", "$90x,90,90,90,90,90#", "$S,0,90#", "$S,7,90#", "$S,1,#", "$#", "$90,90,,90,90,90#"};
    for(unsigned j=0;j<sizeof(bad)/sizeof(bad[0]);j++){
        clean_log();send(&a,bad[j],210);has("$ERR,FORMAT#");
        for(unsigned k=0;k<6;k++)assert(a.target[k]==goal[k]);
    }
    clean_log();send(&a,"$",220);for(unsigned n=0;n<120;n++)send(&a,"1",220);has("$ERR,TOO_LONG#");
    send(&a,"$PING#",221);has("$PONG#");
    clean_log();send(&a,"$90,",230);arm_tick(&a,430);has("$ERR,FRAME_TIMEOUT#");
    send(&a,"$bad$PING#",440);has("$PONG#");
    puts("PASS: wrong field count, seven-field rejection, range, decimals, garbage, overflow, timeout, resync");

    send(&a,"$S,6,100#",450);tick_motion(&a,450,15);
    assert(a.current[5]==1000);for(unsigned k=0;k<5;k++)assert(a.current[k]==goal[k]);
    send(&a,"$180,0,180,0,180,0#",760);tick_motion(&a,760,3);
    send(&a,"$STOP#",821);uint16_t held=a.current[0];arm_tick(&a,900);assert(a.current[0]==held&&!a.moving&&enabled);
    clean_log();send(&a,"$HOME#",901);arm_tick(&a,2901);assert(a.link_lost&&!a.moving&&enabled);has("$WARN,LINK_TIMEOUT,HOLD#");
    send(&a,"$PING#",2902);send(&a,"$HOME#",2903);has("$ERR,DISABLED#");
    send(&a,"$ON#$HOME#",2904);tick_motion(&a,2904,100);for(unsigned k=0;k<6;k++)assert(a.current[k]==900);
    send(&a,"$OFF#",4905);assert(!enabled&&!a.moving);
    puts("PASS: single gripper command, STOP hold, link-loss hold, explicit re-enable, HOME, OFF");

    send(&a,"$ON#$0,180,0,180,0,180#",5000);tick_motion(&a,5000,90);
    for(unsigned k=0;k<6;k++)assert(pulses[k]==(k%2?2000:1000));
    send(&a,"$STATUS#",6801);has("$STATE,1,0,0,0,1800,0,1800,0,1800#");
    arm_receive_fault(&a);assert(a.link_lost&&!a.moving&&enabled);has("$ERR,RX_LOSS#");
    puts("PASS: exact endpoints, status setpoints, receive-loss interlock");

    assert(arm_init(&a,0xfffffff0u));send(&a,"$ON#$100,90,90,90,90,90#",0xfffffff0u);
    arm_tick(&a,4u);assert(a.current[0]==910); /* 32-bit ms rollover */
    for(unsigned i=0;i<20000;i++){
        arm_receive(&a,(uint8_t)(rand()%256),5u+i);
        arm_tick(&a,5u+i);
        for(unsigned k=0;k<6;k++)assert(a.current[k]<=1800 && a.target[k]<=1800);
    }
    puts("PASS: clock wrap and 20000 arbitrary input bytes without out-of-range motion");
    puts("ALL ARM LOGIC TESTS PASS (not a physical-board test)");return 0;
}
