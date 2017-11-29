#include "VisualServoing.hpp"

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <QtWidgets/QVBoxLayout>

#include "vs.h"
#include "ip.h"

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
	//

	//add them to the layout here
    verticalLayout->addWidget(_initButton);
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
}

void VisualServoing::loadMarker2() {

}

void VisualServoing::loadMarker3() {

}

void VisualServoing::detectMarkers() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        rw::kinematics::Frame* cameraFrame = _workcell->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const rw::sensor::Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        cv::Mat im = toOpenCVImage(image);
        cv::Mat imflip;
        cv::flip(im, imflip, 0);
        cv::cvtColor(imflip,imflip,CV_RGB2BGR);

        // Do Image processing here THIS IS COLOR SEGMENTATION
        cv::Mat segmented = ip::segmentateHSV(imflip);
        segmented = ip::opening(segmented, 0 ,3);
        segmented = ip::closing(segmented, 0 ,3);

        std::vector<std::vector<cv::Point> > contours = ip::findContours(segmented);

        segmented = ip::drawContours(contours,segmented);

        //Get the moments
        std::vector<cv::Moments> mu = ip::getMu(contours);

        //get the mass centers
        std::vector<cv::Point2i> mc = ip::getCenterPoints(mu,contours);

        segmented = ip::drawPoints(mc,segmented);

        cv::cvtColor(segmented, segmented, CV_BGR2RGB);

        // Show on QLabel
        QImage img(segmented.data, segmented.cols, segmented.rows, segmented.step, QImage::Format_RGB888);
        QPixmap p = QPixmap::fromImage(img);
        unsigned int maxW = 400;
        unsigned int maxH = 800;
        _processedPicture->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

    }

}


