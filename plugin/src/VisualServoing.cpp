#include "VisualServoing.hpp"

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <QtWidgets/QVBoxLayout>
#include <QDialog>
#include <QtWidgets/QFileDialog>

#include "vs.h"
#include "ip.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#define focal 823.0
#define z 0.5


VisualServoing::VisualServoing():
	RobWorkStudioPlugin("VisualServoing", QIcon(":/pa_icon.png"))
{
	QScrollArea *widg = new QScrollArea(this);
	widg->setWidgetResizable(true);
	QWidget *dockWidgetContent = new QWidget(this);
	QVBoxLayout *verticalLayout = new QVBoxLayout(dockWidgetContent);



	//create stuff we want in the plug in here
    _markerButtons = createMarkerButtons();
    _camPicture = new QLabel();
    _processedPicture = new QLabel();
    _initButton = new QPushButton("Init");
    _background = new QPushButton("Select Background");
	//

	//add them to the layout here
    verticalLayout->addWidget(_initButton);
    verticalLayout->addWidget(_background);
	verticalLayout->addWidget(_markerButtons);
    verticalLayout->addWidget(_camPicture);
    verticalLayout->addWidget(_processedPicture);
    verticalLayout->addStretch(0);
    //verticalLayout->setAlignment(Qt::AlignTop);
	//

	//setup the layouts.
	dockWidgetContent->setLayout(verticalLayout);
	widg->setWidget(dockWidgetContent);
	this->setWidget(widg);
    //

	//_timer = new QTimer(this);
	//connect(snapShot, SIGNAL(pressed()), this, SLOT(capture()));
    connect(_initButton, SIGNAL(pressed()), this, SLOT(init()));
    connect(_background,SIGNAL(pressed()),this, SLOT(loadBackground()));

    /*
	rw::sensor::Image textureImage(300,300, rw::sensor::Image::GRAY, rw::sensor::Image::Depth8U);
	_textureRender = new rwlibs::opengl::RenderImage(textureImage);
	rw::sensor::Image bgImage(0,0, rw::sensor::Image::GRAY, rw::sensor::Image::Depth8U);
	_bgRender = new rwlibs::opengl::RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
     */

}

VisualServoing::~VisualServoing(){
	delete _textureRender;
	delete _bgRender;
}

void VisualServoing::initialize(){
    getRobWorkStudio()->stateChangedEvent().add(boost::bind(&VisualServoing::stateChangedListener, this, _1), this);
}

void VisualServoing::open(rw::models::WorkCell* workcell) {
    _workcell = workcell;
    _state = _workcell->getDefaultState();
}

void VisualServoing::close(){

    // Stop the timer
    //_timer->stop();
    // Remove the texture render
    rw::kinematics::Frame* textureFrame = _workcell->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }
    // Remove the background render
    rw::kinematics::Frame* bgFrame = _workcell->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _workcell = NULL;
}

cv::Mat VisualServoing::toOpenCVImage(const rw::sensor::Image& img) {
    cv::Mat res(img.getHeight(),img.getWidth(), CV_8UC3);
    res.data = (uchar*)img.getImageData();
    return res;
}

void VisualServoing::capture() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        rw::kinematics::Frame* cameraFrame = _workcell->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const rw::sensor::Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        cv::Mat im = toOpenCVImage(image);
        cv::Mat imflip;
        cv::flip(im, imflip, 0);

        // Show on QLabel
        QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 400;
        unsigned int maxH = 800;
        _camPicture->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
    }

}

void VisualServoing::stateChangedListener(const rw::kinematics::State &state){
	_state = state;
    capture();
    detectMarkers();
}

QWidget* VisualServoing::createMarkerButtons(){

    QScrollArea *widg = new QScrollArea();
    widg->setFrameShape(QFrame::NoFrame);
    QWidget* markerButtons = new QWidget();
    QGridLayout *layout = new QGridLayout(markerButtons);

    QToolButton *btns[3];

    btns[0] = new QToolButton();
    btns[0]->setIcon(QIcon(":markers/Marker1.ppm"));
    btns[0]->setIconSize(QSize(65,65));

    btns[1] = new QToolButton();
    btns[1]->setIcon(QIcon(":markers/Marker2a.ppm"));
    btns[1]->setIconSize(QSize(65,65));

    btns[2] = new QToolButton();
    btns[2]->setIcon(QIcon(":markers/Marker3.ppm"));
    btns[2]->setIconSize(QSize(65,65));

    btns[3] = new QToolButton();
    btns[3]->setIconSize(QSize(65,65));

    layout->addWidget(btns[0],0,0);
    layout->addWidget(btns[1],0,1);
    layout->addWidget(btns[2],0,2);
    layout->addWidget(btns[3],0,3);

    connect(btns[0], SIGNAL(pressed()), this, SLOT(loadMarker1()));
    connect(btns[1], SIGNAL(pressed()), this, SLOT(loadMarker2()));
    connect(btns[2], SIGNAL(pressed()), this, SLOT(loadMarker3()));
    connect(btns[3], SIGNAL(pressed()), this, SLOT(testVisualServoing()));

    markerButtons->setLayout(layout);
    widg->setWidget(markerButtons);
    widg->setSizePolicy(QSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed));
    return widg;
}

void VisualServoing::init() {
    // Auto load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("/home/student/Desktop/workspace/PA10WorkCell/ScenePA10RoVi1.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);

    rw::sensor::Image textureImage(300,300, rw::sensor::Image::GRAY, rw::sensor::Image::Depth8U);
    _textureRender = new rwlibs::opengl::RenderImage(textureImage);
    rw::sensor::Image bgImage(0,0, rw::sensor::Image::GRAY, rw::sensor::Image::Depth8U);
    _bgRender = new rwlibs::opengl::RenderImage(bgImage,2.5/1000.0);
    _framegrabber = NULL;

    if (_workcell != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        rw::kinematics::Frame* textureFrame = _workcell->findFrame("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        rw::kinematics::Frame* bgFrame = _workcell->findFrame("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        rw::kinematics::Frame* cameraFrame = _workcell->findFrame("CameraSim");
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new rwlibs::simulation::GLFrameGrabber(width,height,fovy);
                rw::graphics::SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }
    }

    _markerFrame = _workcell->findFrame("Marker");
    _cameraFrame = _workcell->findFrame("Camera");
    _base = _workcell->findFrame("PA10.Base");
    _device = _workcell->findDevice("PA10");

    _UV = vs::calcUV(focal, _markerFrame, _cameraFrame, _base, _state);

    //load mat for the corner marker
    _img_object = imread("/home/student/workspace/RoVi-2017/plugin/markers/Marker3.ppm",cv::IMREAD_COLOR);
    if(_img_object.empty())
        std::cout << "Failed imread(): image not found" << std::endl;

    capture();
}

void VisualServoing::testVisualServoing() {
    std::vector<double> newUV = vs::calcUV(focal, _markerFrame, _cameraFrame, _base, _state);
    std::vector<double> dUV = {newUV[0] - _UV[0], newUV[1] - _UV[1]};

    rw::common::Log::log().info() << "UV: " << _UV[0] << " " << _UV[1] << std::endl;
    rw::common::Log::log().info() << "newUV: " << newUV[0] << " " << newUV[1] << std::endl;
    rw::common::Log::log().info() << "dUV: " << dUV[0] << " " << dUV[1] << std::endl;

    rw::math::Q dq = vs::calcDqFromUV(_UV[0], _UV[1], dUV[0], dUV[1], z, focal, _cameraFrame, _device, _state);
    rw::math::Q newQ = _device->getQ(_state) + dq;

    rw::common::Log::log().info() << "q: " << _device->getQ(_state) << std::endl;
    rw::common::Log::log().info() << "dq: " << dq << std::endl;
    rw::common::Log::log().info() << "newQ: " << newQ << std::endl;

    _device->setQ(newQ, _state);
    getRobWorkStudio()->setState(_state);

    std::vector<double> backUV = vs::calcUV(focal, _markerFrame, _cameraFrame, _base, _state);
    rw::common::Log::log().info() << "backUV: " << backUV[0] << " " << backUV[1] << std::endl;
    std::vector<double> dUVback = {_UV[0] - backUV[0], _UV[1] - backUV[1]};
    rw::common::Log::log().info() << "backUV: " << dUVback[0] << " " << dUVback[1] << std::endl;

    //_UV = newUV;
}

void VisualServoing::loadMarker1() {
    rw::sensor::Image::Ptr image = rw::loaders::ImageLoader::Factory::load("/home/student/workspace/RoVi-2017/plugin/markers/Marker1.ppm");
    _textureRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
    markerMethod = 1;
}

void VisualServoing::loadMarker2() {
    markerMethod = 2;
}

void VisualServoing::loadMarker3() {
    rw::sensor::Image::Ptr image = rw::loaders::ImageLoader::Factory::load("/home/student/workspace/RoVi-2017/plugin/markers/Marker3.ppm");
    _textureRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
    markerMethod = 3;
}

void VisualServoing::loadBackground() {
    QString file = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                "/home",
                                                tr("Images(*.png *.jpg *.ppm)"));
    rw::sensor::Image::Ptr image;
    image = rw::loaders::ImageLoader::Factory::load(file.toStdString());
    _bgRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
}

void VisualServoing::detectMarkers() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        rw::kinematics::Frame* cameraFrame = _workcell->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const rw::sensor::Image& image = _framegrabber->getImage();

    switch(markerMethod) {

        case 1:
        {
            marker1Function(image);
            break;
        }
        case 2:
        {

            break;
        }
        case 3:
        {
            marker3Function(image);
            break;
        }


        default:
        {
            break;
        }
        }
    }

}

void VisualServoing::marker1Function(const rw::sensor::Image& image) {
    // Convert to OpenCV image
    cv::Mat im = toOpenCVImage(image);
    cv::Mat imflip;
    cv::flip(im, imflip, 0);
    cv::cvtColor(imflip, imflip, CV_RGB2BGR);

    cv::Mat segmentedBlue, segmentedRed;

    //FIND BLUE
    segmentedBlue = ip::segmentateHSV(imflip, 120, 121, 250, 256, 100, 256); //blue circles
    segmentedBlue = ip::opening(segmentedBlue, 0, 3);
    segmentedBlue = ip::closing(segmentedBlue, 0, 6);
    segmentedBlue = ip::opening(segmentedBlue, 0, 14);

    std::vector<std::vector<cv::Point> > contoursBlue = ip::findContours(segmentedBlue);

    segmentedBlue = ip::drawContours(contoursBlue, segmentedBlue);

    //Get the moments
    std::vector<cv::Moments> muBlue = ip::getMu(contoursBlue);

    //get the mass centers
    std::vector<cv::Point2i> mcBlue = ip::getCenterPoints(muBlue, contoursBlue);

    //FIND RED
    segmentedRed = ip::segmentateHSV(imflip, 0, 1, 250, 256, 100, 256); //blue circles
    segmentedRed = ip::opening(segmentedRed, 0, 3);
    segmentedRed = ip::closing(segmentedRed, 0, 6);
    segmentedRed = ip::opening(segmentedRed, 0, 14);

    std::vector<std::vector<cv::Point> > contoursRed = ip::findContours(segmentedRed);

    segmentedRed = ip::drawContours(contoursRed, segmentedRed);

    //Get the moments
    std::vector<cv::Moments> muRed = ip::getMu(contoursRed);

    //get the mass centers
    std::vector<cv::Point2i> mcRed = ip::getCenterPoints(muRed, contoursRed);

    //segmentedRed = ip::drawPoints(mcRed, segmentedRed,cv::Vec3b(0,0,255));

    //DO STUFF WITH RED AND BLUE
    cv::Mat result = segmentedBlue + segmentedRed;

    std::vector<cv::Point2i> bluePoints = ip::decideOnBlueMarkers(mcBlue, mcRed);

    if (bluePoints.size() < 3 || mcRed.size() < 1) {
        rw::common::Log::log().info() << "Did not find enough markers" << std::endl;
        return;
    }

    std::vector<cv::Point2i> allPoints;

    for (int i = 0; i < mcRed.size(); i++) {
        cv::circle(result, mcRed[i], 10, cv::Vec3b(255, 0, 0), 4);
        allPoints.push_back(mcRed[i]);
    }
    for (int i = 0; i < bluePoints.size(); i++) {
        allPoints.push_back(bluePoints[i]);
        cv::circle(result, bluePoints[i], 10, cv::Vec3b(0, 0, 255), 4);
    }

    allPoints = ip::toRobotPoints(allPoints, result);

    // Show on QLabel
    QImage img(result.data, result.cols, result.rows, result.step, QImage::Format_RGB888);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _processedPicture->setPixmap(p.scaled(maxW, maxH, Qt::KeepAspectRatio));
}

void VisualServoing::marker3Function(const rw::sensor::Image& image) {
    cv::Mat im = toOpenCVImage(image);
    cv::Mat imflip;
    cv::flip(im, imflip, 0);
    cv::cvtColor(imflip, imflip, CV_RGB2GRAY);

    cv::Mat img_scene = imflip;

    cv::Mat img_object = _img_object.clone();

    if( !img_object.data || !img_scene.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return; }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;

    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );

    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;


    //-- Step 2: Calculate descriptors (feature vectors)
    cv::Mat descriptors_object, descriptors_scene;

    detector->detectAndCompute( img_object, cv::Mat() ,keypoints_object, descriptors_object );
    detector->detectAndCompute( img_scene, cv::Mat(), keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );

    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ )
    { double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ )
    {
        if( matches[i].distance < 3*min_dist)
            good_matches.push_back( matches[i]);
    }

    cv::Mat img_matches;

    //-- Localize the object
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;

    for( int i = 0; i < good_matches.size(); i++ )
    {
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }

    cv::Mat H = findHomography( obj, scene, cv::RANSAC );

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( img_object.cols, 0 );
    obj_corners[2] = cvPoint( img_object.cols, img_object.rows );
    obj_corners[3] = cvPoint( 0, img_object.rows );
    std::vector<cv::Point2f> scene_corners(4);

    perspectiveTransform( obj_corners, scene_corners, H);

    std::vector<cv::Point2i> points;
    points.push_back(scene_corners[0] );
    points.push_back(scene_corners[1] );
    points.push_back(scene_corners[2] );
    points.push_back(scene_corners[3] );

    /*for(int i = 0; i < points.size(); i++){
       std::cout << i << " " <<points[i] << std::endl;
    }*/

    for (int i = 0; i < points.size(); i++) {
        cv::circle(img_scene,points[i],10,0,4);
    }

    // Show on QLabel
    QImage img(img_scene.data, img_scene.cols, img_scene.rows, img_scene.step, QImage::Format_Grayscale8);
    QPixmap p = QPixmap::fromImage(img);
    unsigned int maxW = 400;
    unsigned int maxH = 800;
    _processedPicture->setPixmap(p.scaled(maxW, maxH, Qt::KeepAspectRatio));
}


