#include <iostream>
#include <signal.h>
#include <string>
#include <stdlib.h>
#include <ros/ros.h>
#include <pthread.h>

using namespace std;

void *Thread_other(void *)
{
    while(1) 
    {
        cout << "----------------------" << endl;
        cout << "please enter '1' or '2' to determine which agv are you want to startup." << endl;
        cout << "first enter '1' or '2',then enter 'enter'." << endl;
        cout << "----------------------" << endl;
        char agv_num;
        cin >> agv_num;
        if(agv_num == '1')
        {
            char char1[50] = "$(rospack find rvizCarControl)/launch/agv1.sh";
            system(char1);
        }
        else if(agv_num == '2')
        {
            char char1[50] = "$(rospack find rvizCarControl)/launch/agv2.sh";
            system(char1);
        }
        else
        {
            cout << "input error!!!" << endl;
            cout << "please reboot the program to start." << endl;
        }
    }
}

void Pthread_other()
{
  pthread_t pid_topic;
  printf("usart thread start...\n");
  int pthread_spin = pthread_create(&pid_topic, NULL,Thread_other, NULL);
}

int main(int argc,char** argv)
{
    //while(1)
    {
        // cout << "enter to create a other pthread." << endl;
        // char ccc;
        // cin >> ccc;
        // Pthread_other();
        cout << "----------------------" << endl;
        cout << "please enter '1' or '2' to determine which agv are you want to startup." << endl;
        cout << "first enter '1' or '2',then enter 'enter'." << endl;
        cout << "----------------------" << endl;
        char agv_num;
        cin >> agv_num;
        if(agv_num == '1')
        { 
            char char1[50] = "$(rospack find rvizCarControl)/launch/agv1.sh";
            system(char1);
        }
        else if(agv_num == '2')
        {
            char char1[50] = "$(rospack find rvizCarControl)/launch/agv2.sh";
            system(char1);
        }
        else
        {
            cout << "input error!!!" << endl;
            cout << "please reboot the program to start." << endl;
        }
    }
    return 0;
}