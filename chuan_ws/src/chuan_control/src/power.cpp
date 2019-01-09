// //xx
//https://github.com/derekmolloy/boneDeviceTree/
//  https://www.cnblogs.com/einstein-2014731/p/6876580.html
// //键盘键 ‘1’ ‘2’  控制电源开关
#include "stdio.h"
#include "stdlib.h"
#include <iostream>
#include <simplegpio.h>
using namespace std;

const int GPIO_PU6_P58 = 166;   //第3排左数第6个
//const int GPIO_PH1_P50 = 57;


int main(void)
{
//    gpio_export(GPIO_PH1_P50);
    gpio_export(GPIO_PU6_P58);
 //   gpio_set_dir(GPIO_PH1_P50, INPUT_PIN);
    gpio_set_dir(GPIO_PU6_P58, OUTPUT_PIN);
    char ch;
    while(1)
    {
        printf("please input:1:on or 2:off\n");
        ch = getchar();
        if(ch == '1')
        {
        printf("power on\n");
        gpio_set_value(GPIO_PU6_P58, HIGH);
        }
        if(ch == '2')
        {
        printf("power  off\n");
        sleep(2);
        gpio_set_value(GPIO_PU6_P58, LOW);
        }
    }
 return 0;
}
