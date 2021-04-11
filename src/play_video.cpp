#include <play_video.h>

void PlayVideo(std::string video_path, std::string video_name, std::string window_name){
    /*
    Play the video
    */

    VideoCapture cap(video_path + video_name);

    // if not success, exit program
    if (cap.isOpened() == false)  
    {
        cout << "Cannot open the video file" << endl;
        cin.get(); //wait for any key press
        return;
    }

    //Uncomment the following line if you want to start the video in the middle
    //cap.set(CAP_PROP_POS_MSEC, 300); 

    //get the frames rate of the video
    double fps = cap.get(CAP_PROP_FPS); 
    cout << "Frames per seconds : " << fps << endl;

    namedWindow(window_name, WINDOW_NORMAL); //create a window

    while (true)
    {
        Mat frame;
        bool bSuccess = cap.read(frame); // read a new frame from video 

        //Breaking the while loop at the end of the video
        if (bSuccess == false) 
        {
            cout << "Found the end of the video" << endl;
            break;
        }

        //show the frame in the created window
        imshow(window_name, frame);

        //wait for for 10 ms until any key is pressed.  
        //If the 'Esc' key is pressed, break the while loop.
        //If the any other key is pressed, continue the loop 
        //If any key is not pressed withing 10 ms, continue the loop
        if (waitKey(10) == 27)
        {
            cout << "Esc key is pressed by user. Stoppig the video" << endl;
            break;
        }

        
    }
    cap.release();
    destroyAllWindows();

}

