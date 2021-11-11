/*
** listener.c -- a datagram sockets "server" demo
*/


/*
 * Copyright (c) 2019 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include "common.hh"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pthread.h> 
#include <time.h> 
using namespace std;
using namespace tinyxml2;



namespace {
const char* about = "Pose estimation of ArUco marker images";
const char* keys  =
        "{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        ;
}

void SetupRobots();


#define HELLOPORT "4241"
#define MYPORT "4242"   // the port users will be connecting to
#define MAXBUFLEN 255
#define SAMPLINGTIME 100000

// get sockaddr, IPv4 or IPv6:
 char buf[MAXDATASIZE];

//----prototipos-----//
int comRobot(int id,string ip,string port,int instruction);
void tokenize(const string s, char c,vector<string>& v);
void concatenateChar(char c, char *word);
void operationSend();
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }
    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}
void SetupRobots();
enum {r1, r2, r3, r4,r5};
     //definition of robots
Robot robot1,robot2,robot3,robot4;//se define la clase para los distintos robots.
struct record_data
{
    std::ostringstream vector_to_marker;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    std::vector<cv::Vec3d> rvecs, tvecs;
};
struct record_data data;//shared data bewtwen threads
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
/*void *recordAruco(void *threadArg)
{
   struct record_data *thread_data=(record_data*)malloc(sizeof(struct record_data));

   thread_data=(struct record_data*) &threadArg;
    while (thread_data->in_video.grab())
    {
        thread_data->in_video.retrieve(thread_data->image);
        thread_data->image.copyTo(thread_data->image_copy);
        //std::vector<int> ids;
        //std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(thread_data->image,thread_data-> dictionary,thread_data-> corners,thread_data-> ids);
        // if at least one marker detected
        
        if (thread_data->ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(thread_data->image_copy, 
                thread_data->corners,thread_data-> ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(thread_data->corners,thread_data->marker_lenght_m,
                    thread_data->camera_matrix, thread_data->dist_coeffs, rvecs, tvecs);
                    
            std::cout << "Translation: " << tvecs[0]
                << "\tRotation: " << rvecs[0] 
                << std::endl;
            
            // Draw axis for each marker
            for(int i=0; i < thread_data->ids.size(); i++)
            {
                
                cv::aruco::drawAxis(thread_data->image_copy, thread_data->camera_matrix,thread_data-> dist_coeffs,
                        rvecs[i], tvecs[i], 0.1);
                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
                thread_data->vector_to_marker.str(std::string());
                thread_data->vector_to_marker << std::setprecision(4)
                                << "x: " << std::setw(8) << tvecs[0](0);
                cv::putText(thread_data->image_copy, thread_data->vector_to_marker.str(),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                thread_data->vector_to_marker.str(std::string());
                thread_data->vector_to_marker << std::setprecision(4)
                                << "y: " << std::setw(8) << tvecs[0](1);
                cv::putText(thread_data->image_copy, thread_data->vector_to_marker.str(),
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                thread_data->vector_to_marker.str(std::string());
                thread_data->vector_to_marker << std::setprecision(4)
                                << "z: " << std::setw(8) << tvecs[0](2);
                cv::putText(thread_data->image_copy, thread_data->vector_to_marker.str(),
                            cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);
            }
        }
        imshow("Pose estimation", thread_data->image_copy);
        
        char key = (char)cv::waitKey(thread_data->wait_time);
        if (key == 27)
            break;
    }
    thread_data->in_video.release();
    return NULL;
}*/

void *dataAruco(void *arg){
    int id,instruction;
    string ip,port;
    robot1.SetupConection(id,ip,port);//for now only use 1 robot for communication
    //in this case the experiment needs the velocity 
    struct timeval tval_before, tval_after, tval_sample;
    tval_sample.tv_sec=0;
    tval_sample.tv_usec=0;
    long int deltaTime=static_cast<long int>(tval_sample.tv_usec);
    while(true){
        
        gettimeofday(&tval_before,NULL);
        if(deltaTime == SAMPLINGTIME){
            instruction=OP_VEL_ROBOT;
            comRobot(id,ip,port,instruction);

        }
        else if( deltaTime<0)
        {
            //error
        }
        else{
            
        }

        gettimeofday(&tval_after,NULL);

        timersub(&tval_after,&tval_before,&tval_sample);
        deltaTime=static_cast<long int>(tval_sample.tv_usec);

    }

    return NULL;
}
int main(int argc,char **argv)
{
    pthread_t detectAruco;
    

    cv::CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return 1;
    }

    if (parser.get<bool>("h")) {
        parser.printMessage();
        return 0;
    }

    int dictionaryId = parser.get<int>("d");

    float marker_length_m = parser.get<float>("l");
    int wait_time = 10;
    if (marker_length_m<= 0) {
        std::cerr << "marker length must be a positive value in meter" 
                  << std::endl;
        return 1;
    }
    cv::String videoInput = "0";//se selecciona la entrada de la camara
    cv::VideoCapture in_video;
    if (parser.has("v")) {
        videoInput = parser.get<cv::String>("v");
        if (videoInput.empty()) {
            parser.printMessage();
            return 1;
        }
     char* end = nullptr;
        int source = static_cast<int>(std::strtol(videoInput.c_str(), &end, \
            10));
        if (!end || end == videoInput.c_str()) {
            in_video.open(videoInput); // url
            in_video.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
			in_video.set(cv::CAP_PROP_FPS,30);
        } else {
            in_video.open(source); // id
        }
    } else {
        in_video.open(0);
    }

    if (!parser.check()) {
        parser.printErrors();
        return 1;
    }
    if (!in_video.isOpened()) {
        std::cerr << "failed to open video input: " << videoInput << std::endl;
        return 1;
    }

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;
    std::ostringstream vector_to_marker;

    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    
    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;
    //pthread_create(&record,NULL,recordAruco,(void*)&data);
   
    pthread_create(&detectAruco,NULL,dataAruco,NULL);
    while (in_video.grab())
    {
        in_video.retrieve(image);
        image.copyTo(image_copy);
        //std::vector<int> ids;
        //std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(image, dictionary, data.corners, data.ids);

        // if at least one marker detected
        if (data.ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(image_copy, data.corners, data.ids);
            //std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(data.corners, marker_length_m,
                    camera_matrix, dist_coeffs, data.rvecs, data.tvecs);
                    
            /*std::cout << "Translation: " << tvecs[0]
                << "\tRotation: " << rvecs[0] 
                << std::endl;
            */
            // Draw axis for each marker
            for(int i=0; i < data.ids.size(); i++)
            {
                cv::aruco::drawAxis(image_copy, camera_matrix, dist_coeffs,
                        data.rvecs[i], data.tvecs[i], 0.1);

                // This section is going to print the data for all the detected
                // markers. If you have more than a single marker, it is
                // recommended to change the below section so that either you
                // only print the data for a specific marker, or you print the
                // data for each marker separately.
                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "x: " << std::setw(8) << data.tvecs[0](0);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "y: " << std::setw(8) << data.tvecs[0](1);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);

                vector_to_marker.str(std::string());
                vector_to_marker << std::setprecision(4)
                                 << "z: " << std::setw(8) << data.tvecs[0](2);
                cv::putText(image_copy, vector_to_marker.str(),
                            cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 252, 124), 1, CV_AVX);
            }
        }

        imshow("Pose estimation", image_copy);
        char key = (char)cv::waitKey(wait_time);
        if (key == 27)
            break;
    }
     in_video.release();

    /*while(1){//main loop

         //primero se elige al robot al que se desea mandar una determinada instruccion
        cout<<"(0)robot1"<<endl<<"(1)robot2"<<endl<<"(2)robot3"<<endl<<"(3)robot4"<<endl;
        cout<<"eliga el robot con el cual desea comunicarse"<<endl;
        cin>>cont;

        switch (cont)
        {
        case r1 :
            
            robot1.SetupConection(id,ip,port);
            break;
        
        case r2 :
            robot2.SetupConection(id,ip,port);
            break;

        
        case r3 :
            robot3.SetupConection(id,ip,port);
            break;

        
        case r4 :
            robot4.SetupConection(id,ip,port);
            break;
            
        default:
            break;
        
        }
        comRobot(id,ip,port);
        
    }*/
    pthread_exit(NULL);
    return 0;
}

void SetupRobots()
{
    // Read the sample.xml file
    XMLDocument Robotdoc;
    Robotdoc.LoadFile( "robots_info.xml" );

    XMLNode* Robotarium =Robotdoc.FirstChild();
    XMLElement *robot=Robotarium->FirstChildElement("robot");
    int i=0;
    while(robot !=NULL)
    {
        
        XMLElement *robotChild=robot->FirstChildElement("ID");
        int ID;
        robotChild->QueryIntText(&ID);
        cout<<"ID:"<<ID<<endl;
    
         robotChild=robot->FirstChildElement("IP");
         const char* ip=robotChild->GetText();
         string ss=ip;
         cout<<"ip:"<<ip<<endl;

        robotChild=robot->FirstChildElement("PORT");
        const char* port=robotChild->GetText();
        string p=port;
        cout<<"puerto:"<<p<<endl;
        
        robot=robot->NextSiblingElement("robot"); 
        switch (i)
        {
            case 0:
                robot1.SetupRobotData(ID,ss,p);
                break;
            case 1:
                robot2.SetupRobotData(ID,ss,p);
                break;
            case 2:
                robot3.SetupRobotData(ID,ss,p);
                break;
             case 3:
                robot4.SetupRobotData(ID,ss,p);
                break;

        }       
        i++;   
    }
}

int comRobot(int id,string ip,string port,int instruction){
    
    //se crea el socket y se establece la comunicación
    int sockfd;
    struct addrinfo hints, *servinfo, *p;
    int rv;
    int numbytes;
    struct sockaddr_storage robot_addr;
    cout<<"puert Robot:"<<port<<endl;
    socklen_t addr_len = sizeof robot_addr;


    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // set to AF_INET to force IPv4
    hints.ai_socktype = SOCK_DGRAM;
   //hints.ai_flags = IPPROTO_UDP; 

    const char *ipRobot=ip.c_str();
    const char *portRobot=port.c_str();
    
    if ((rv = getaddrinfo(ipRobot, portRobot, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
    }
    // loop through all the results and make a socket
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
            p->ai_protocol)) == -1) {
            perror("talker: socket");
            continue;
        }
    break;
   
    }
    if (p == NULL) {
        fprintf(stderr, "talker: failed to create socket\n");
        return 2;
    }
    memset (buf, '\0', MAXDATASIZE); /* Pone a cero el buffer inicialmente */
    //aqui se indica la operacion que se desea realizar
    operation_send.id=id;//se asigna el id del robot1
    string data;
    string delimiter=":";
    /*cout<<"elige operacion"<<endl;//se debe ingresar la operacion y los correspondientes datos
    operationSend();//se elige la operacion a enviar
    if(operation_send.op != OP_SALUDO && operation_send.op != OP_VEL_ROBOT){
        //se ingresa la informacion correspondiente a la operacion elegida.
        cout<<"ingresa datos: ";
        cin.ignore();
        cin>>operation_send.data;
        operation_send.len = strlen (operation_send.data);
    }*/
    operation_send.op=instruction;

    if ((numbytes = sendto(sockfd,(char *) &operation_send, operation_send.len+HEADER_LEN, 0,p->ai_addr, p->ai_addrlen)) == -1) {
        perror("talker: sendto");
        exit(1);
    }

    cout<<"mensaje enviado"<<endl;

    if((numbytes=recvfrom(sockfd,buf,MAXBUFLEN-1,0,(struct sockaddr*)&robot_addr, &addr_len))==-1){
    }
    operation_recv=( struct appdata*)&buf;
    if((numbytes< HEADER_LEN) || (numbytes != operation_recv->len+HEADER_LEN) ){

        cout<<"(servidor) unidad de datos incompleta\n";
    }
    else{
        
        cout<<"(servidor) id "<<operation_recv->id;
        cout<<" operacion solicitada [op 0x]"<<operation_recv->op;
        cout<<" contenido "<<operation_recv->data<<endl;
    }
    // relaiza operacion solicitada por el cliente 

    switch (operation_recv->op){
        case OP_SALUDO:
            cout<<" contenido "<<operation_recv->data<<endl;
        break;
        case OP_MESSAGE_RECIVE:
            cout<<" contenido "<<operation_recv->data<<endl;
        break;
        case OP_VEL_ROBOT:
            data=operation_recv->data;
            char del =',';
            vector<string> speed;
            tokenize(data,del,speed);
            cout<<"velocidad rueda derecha: "<<speed[0]<<endl;
            cout<<"velocidad rueda izquierda: "<<speed[1]<<endl;
            break;

    }
    
    freeaddrinfo(servinfo);
    
    close(sockfd);
    
    return 0;
    


}
void tokenize(const string s, char c,vector<string>& v)//sirve para separa la entrada string.
{
   string::size_type i = 0;
   string::size_type j = s.find(c);

   while (j != string::npos) 
   {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == string::npos)
         v.push_back(s.substr(i, s.length()));
   }
}

void concatenateChar(char c, char *word){
    char cadenaTemporal[2];
    cadenaTemporal[0] = c;
    cadenaTemporal[1] = '\0';
    strcat(word, cadenaTemporal);

}
void operationSend(){
    int value;
    cout<<"(0) SALUDO"<<endl;
    cout<<"(1) MOVE_WHEEL"<<endl;
    cout<<"(2) STOP_WHEEL"<<endl;
    cout<<"(3) MESSAGE"<<endl;
    cout<<"(4) TELEMETRY"<<endl;
    cout<<"(5) VEL_wheel"<<endl;
    cin>>value;
    switch (value)
    {

        case 0:
            operation_send.op=OP_SALUDO;
            strcpy(operation_send.data,"Saludo");
            operation_send.len = strlen (operation_send.data);
            break;
        case 1:
            operation_send.op=OP_MOVE_WHEEL;
            break;
        case 2:
            operation_send.op=OP_STOP_WHEEL;
            break;
        case 3:
            break;
        case 4:
            break;
        case 5:
            operation_send.op=OP_VEL_ROBOT;
            strcpy(operation_send.data,"velocidad");
            operation_send.len = strlen (operation_send.data);
            break;
    }

    
}
