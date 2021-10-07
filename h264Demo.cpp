#include "h264_decoder.h"
#include <iostream>
#include <thread>
#include <string>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include "PracticalSocket.h" // For UDPSocket and SocketException
#include <cstdlib>           // For atoi()
#include "config.h"

#define BUF_LEN 65540 // Larger than maximum UDP packet size
#define SERV_PORT   9999

using namespace cv;
using namespace std;
using std::thread;

int sock_fd;
int len;
bool set_rect =false;
string command;


cv::Mat org;
cv::Point pre_pt;
cv::Point cur_pt;

typedef struct
{
    int target_header;

    int target_id[20];
    int target_x[20];
    int target_y[20];
    int target_w[20];
    int target_h[20];
    int target_num;
    float target_velocity[20];
} targetInfo;

typedef struct
{
    bool use_hdr;
    bool use_flip;
} controlData;

int sendSocketData(){
    controlData tempData;
    tempData.use_flip = true;
    tempData.use_hdr = true;

    struct sockaddr_in send_addr;
    memset(&send_addr,0,sizeof(struct sockaddr_in));
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(9999);
    send_addr.sin_addr.s_addr = inet_addr("192.168.1.128");
    if(sendto(sock_fd,(controlData*)&tempData,sizeof(tempData),0,(sockaddr *)&send_addr,sizeof(send_addr)) <= 0)
    {
        printf("send data error");
    }
}


int getSocketData(targetInfo temp){
    int recv_num;
    // char recv_buf[20];
    struct sockaddr_in addr_client;
    recv_num = recvfrom(sock_fd, (targetInfo*)&temp, sizeof(temp), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);

    if(recv_num < 0)
    {
        perror("recvfrom error:");
        exit(1);
        return 0;
    }
    if(temp.target_header==0xFFEEAABB) {
        std::cout<<"recv data: "<<std::endl;
        for(int i =0; i<temp.target_num;i++){
            std::cout<<temp.target_id[i]<<", ";
            std::cout<<temp.target_x[i]<<", ";
            std::cout<<temp.target_y[i]<<", ";    
            std::cout<<temp.target_w[i]<<", ";
            std::cout<<temp.target_h[i]<<", ";
        }
        std::cout<<std::endl;
        return 1;
    }
}


int initSocketData(){
   sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock_fd < 0)
    {
        perror("socket");
        exit(1);
    }

    struct sockaddr_in addr_serv;

    memset(&addr_serv, 0, sizeof(struct sockaddr_in)); 
    addr_serv.sin_family = AF_INET; 
    addr_serv.sin_port = htons(SERV_PORT); 
    addr_serv.sin_addr.s_addr = htonl(INADDR_ANY); 
    len = sizeof(addr_serv);

    if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
    {
        perror("bind error:");
        exit(1);
    }else  return 0;
}



void keboardListener()
{
	while(1)
    {
        cin>>command;
        cout<<"command:"<<command<<endl;
    }
}

void on_mouse(int event, int x, int y, int flags, void *)
{
    cv::Mat dst, img, tmp;

    if (event == CV_EVENT_LBUTTONDOWN)
    {
        pre_pt = cv::Point(x, y);
        cout<<pre_pt<<endl;
    }
    else if (event == CV_EVENT_MOUSEMOVE && flags)//摁下左键，flags为1 

    {
        org.copyTo(tmp);
        cur_pt = cv::Point(x, y);
        cv::circle(tmp,cur_pt,4,Scalar(0, 255, 0, 0),2,8);
        cv::rectangle(tmp, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);
        std::cout<<cur_pt<<endl;
        cv::imshow("CamShow", tmp);//画的时候显示框
    }
    else if (event == CV_EVENT_LBUTTONUP &&  abs(pre_pt.x - cur_pt.x)>10 && abs(cur_pt.y- pre_pt.y)>10 && abs(pre_pt.x - cur_pt.x)<org.cols && abs(cur_pt.y- pre_pt.y)<org.rows && cur_pt.x>0 && cur_pt.y>0)
    {
        set_rect = true;
        org.copyTo(img);
        // cv::rectangle(img, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 1, 8, 0);
        // cv::imshow("CamShow", img);//画完后显示框
        int width = abs(pre_pt.x - cur_pt.x);
        int height = abs(pre_pt.y - cur_pt.y);
        dst = org(Rect(min(cur_pt.x, pre_pt.x), min(cur_pt.y, pre_pt.y), width, height));
        cv::resize(dst,dst,cv::Size(width*1.5,height*1.5));
        cv::namedWindow("dst");
        cvMoveWindow("dst",0,560);
        cv::imshow("dst", dst);

    }

}


int main(int argc, char * argv[]) {

	targetInfo get_target_data;
    initSocketData();

    CH264Decoder m_h264Decoder;
    m_h264Decoder.initial();

    unsigned short servPort = atoi("10000"); // First arg:  local port

    thread keyboardListenerTh(keboardListener);
    namedWindow("CamShow");
    setMouseCallback("CamShow", on_mouse, 0);   
    cvMoveWindow("CamShow",20,20);

    try {
        UDPSocket sock(servPort);

        unsigned char buffer[BUF_LEN]; // Buffer for echo string
        int recvMsgSize; // Size of received message
        string sourceAddress; // Address of datagram source
        unsigned short sourcePort; // Port of datagram source

        clock_t last_cycle = clock();
        int screenshotCnt = 1;


        while (1) {
            // Block until receive message from a client
            do {
                recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
            } while (recvMsgSize > sizeof(int));
            int total_pack = ((int * ) buffer)[0];

            cout << "expecting length of packs:" << total_pack << "Pack size: "<<PACK_SIZE<< endl;
            unsigned char * longbuf = new unsigned char[PACK_SIZE * total_pack];
            for (int i = 0; i < total_pack; i++) {
                recvMsgSize = sock.recvFrom(buffer, BUF_LEN, sourceAddress, sourcePort);
                if (recvMsgSize != PACK_SIZE) {
                    cerr << "Received unexpected size pack:" << recvMsgSize << endl;
                    //continue;
                }
                memcpy( & longbuf[i * PACK_SIZE], buffer, PACK_SIZE);
            }

            cout << "Received packet from " << sourceAddress << ":" << sourcePort << endl;
            sendSocketData();
    
            m_h264Decoder.decode(longbuf, PACK_SIZE * total_pack,org);

            if (org.size().width == 0) {
                cerr << "decode failure!" << endl;
                continue;
            }
            cv::resize(org, org, cv::Size(1520,560));
            if(set_rect)  {
                std::cout<<"set rect!"<<std::endl;
                cv::rectangle(org, pre_pt, cur_pt, Scalar(0, 255, 0, 0), 2, 8, 0);
            }
             imshow("CamShow",org);

            //imwrite("frame.png",img);
            if(command == "p")
            {
                string filename;
                std::stringstream StrStm;
                StrStm<<screenshotCnt++;
                StrStm>>filename;
                imwrite("img"+filename+".png",org);
                command = "";
            }
            
            getSocketData(get_target_data);

            free(longbuf);

            cv::waitKey(30);
        }
    } catch (SocketException & e) {
        cerr << e.what() << endl;
        exit(1);
    }

    return 0;
}







