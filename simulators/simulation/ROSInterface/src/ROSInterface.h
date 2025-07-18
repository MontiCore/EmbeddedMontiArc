#pragma once

#include <ros/ros.h>
#include <jni.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>

using namespace std;
class ROSInterface{

    ros::Subscriber action_subscriber;
    ros::Subscriber reset_subscriber;
    ros::Publisher state_publisher;
    ros::Publisher terminate_publisher;
    ros::Publisher reward_publisher;
    
    ros::Subscriber action2_subscriber;
    ros::Subscriber reset2_subscriber;
    ros::Publisher state2_publisher;
    ros::Publisher terminate2_publisher;

    jclass cls;
    JNIEnv* env;
    jobject* obj;
    jmethodID actionMethodID;
    jmethodID resetMethodID;
    jmethodID reset2MethodID;
    jmethodID action2MethodID;

    public:
    ROSInterface(JNIEnv* envir, jobject* objec){
        env = envir;
        obj = objec;
    }

    void init(){
        char* tmp = strdup("");
        int i = 0;
        ros::init(i, &tmp, "MontiSimROS");
        ros::NodeHandle node_handle = ros::NodeHandle();

        action_subscriber = node_handle.subscribe("/sim/step",5,&ROSInterface::sim_stepCallback, this, ros::TransportHints().tcpNoDelay());
        reset_subscriber = node_handle.subscribe("/sim/reset",5,&ROSInterface::sim_resetCallback, this, ros::TransportHints().tcpNoDelay());

        state_publisher = node_handle.advertise<std_msgs::Float32MultiArray>("/sim/state", 5);
        terminate_publisher = node_handle.advertise<std_msgs::Bool>("/sim/terminal",5);
        reward_publisher = node_handle.advertise<std_msgs::Float32>("/sim/reward",5);

        action2_subscriber = node_handle.subscribe("/sim/step2",5, &ROSInterface::sim_step2Callback, this, ros::TransportHints().tcpNoDelay());
        reset2_subscriber = node_handle.subscribe("/sim/reset2",5, &ROSInterface::sim_reset2Callback, this, ros::TransportHints().tcpNoDelay());
        
        state2_publisher = node_handle.advertise<std_msgs::Float32MultiArray>("/sim/state2", 5);
        terminate2_publisher = node_handle.advertise<std_msgs::Bool>("/sim/terminal2",5);

        cls = env->GetObjectClass(*obj);
        actionMethodID = env->GetMethodID(cls, "action", "([F)V");
        resetMethodID = env->GetMethodID(cls, "reset1", "(Z)V");
        reset2MethodID = env->GetMethodID(cls, "reset2", "(Z)V");
        action2MethodID = env->GetMethodID(cls, "action2", "([F)V");

        jmethodID setCppInterfaceID = env->GetMethodID(cls, "setCppInterface", "(J)V");

        env->CallVoidMethod(*obj, setCppInterfaceID, (jlong) this);
        ros::spin();
    }

    void sim_stepCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        int msg_size = msg->data.size();
        float *message = new float[msg_size];

        for(int i = 0; i<msg_size;++i){
            message[i] = msg->data[i];
        }
        
        jfloatArray arr = (env)->NewFloatArray(msg_size);
        env->SetFloatArrayRegion(arr, 0, msg_size, message);
        env->CallVoidMethod(*obj, actionMethodID, arr);
    }

    void sim_resetCallback(const std_msgs::Bool::ConstPtr& msg){
        bool term = msg->data;
        if(!term) return;
        jboolean reset_ = (jboolean) term;
        env->CallVoidMethod(*obj, resetMethodID, reset_);
    }

    void sim_step2Callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
        int msg_size = msg->data.size();
        float *message = new float[msg_size];

        for(int i = 0;i<msg_size;++i){
            message[i] = msg->data[i];
        }

        jfloatArray arr = env->NewFloatArray(msg_size);
        env->SetFloatArrayRegion(arr, 0, msg_size, message);
        env->CallVoidMethod(*obj, action2MethodID, arr);
    }

    void sim_reset2Callback(const std_msgs::Bool::ConstPtr& msg){
        bool term = msg->data;
        if(!term) return;
        jboolean reset_ = (jboolean) term;
        env->CallVoidMethod(*obj, reset2MethodID, reset_);
    }

    void publish(jfloatArray temp, jboolean terminal){
        int len = env->GetArrayLength(temp);
        float *state = new float[len];
        state = env->GetFloatArrayElements(temp, NULL);
        bool term = (bool) terminal;
        std_msgs::Bool term_msg;
        std_msgs::Float32MultiArray state_msg;
        term_msg.data = term;
        state_msg.data.resize(len);

        for(int i = 0; i<len;++i){
            state_msg.data[i] = state[i];
        }

        state2_publisher.publish(state_msg);
        terminate2_publisher.publish(term_msg);
    }

    void publish(jfloatArray temp, jboolean terminal, jfloat reward){
        int len = env->GetArrayLength(temp);
        float *state = new float[len];
        state = env->GetFloatArrayElements(temp, NULL);

        std_msgs::Bool term_msg;
        std_msgs::Float32 reward_msg;
        std_msgs::Float32MultiArray state_msg;
        term_msg.data = (bool) terminal;
        reward_msg.data = (float) reward;
        state_msg.data.resize(len);

        for(int i = 0; i<len;++i){
            state_msg.data[i] = state[i];
        }

        state_publisher.publish(state_msg);
        terminate_publisher.publish(term_msg);
        reward_publisher.publish(reward_msg);
    }
};

extern "C"
{
JNIEXPORT void JNICALL Java_de_rwth_montisim_simulation_simulator_RLSimulationHandler_startros
(JNIEnv* env, jobject obj) {
    ROSInterface *interf = new ROSInterface(env, &obj);
    (*interf).init();
}

JNIEXPORT void JNICALL Java_de_rwth_montisim_simulation_simulator_RLSimulationHandler_publishNonTrainMessage
(JNIEnv* env, jobject obj, jlong cppobj, jfloatArray state, jboolean terminal) {
    ROSInterface *interf = (ROSInterface*) cppobj;
    (*interf).publish(state, terminal);
}

JNIEXPORT void JNICALL Java_de_rwth_montisim_simulation_simulator_RLSimulationHandler_publishTrainMessage
(JNIEnv* env, jobject obj, jlong cppobj, jfloatArray state, jboolean terminal, jfloat reward){
    ROSInterface *interf = (ROSInterface*) cppobj;
    (*interf).publish(state, terminal, reward);
}
}