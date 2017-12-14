#include "VisualServoing.hpp"

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <QtWidgets/QVBoxLayout>
#include <QDialog>
#include <QtWidgets/QFileDialog>

#include "vs.h"
#include "ip.h"
#include "vsMult.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"


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

void VisualServoing::capture() {
    if (_framegrabber != NULL) {
        // Get the image as a RW image
        rw::kinematics::Frame* cameraFrame = _workcell->findFrame("CameraSim");
        _framegrabber->grab(cameraFrame, _state);
        const rw::sensor::Image& image = _framegrabber->getImage();

        // Convert to OpenCV image
        cv::Mat im = ip::toOpenCVImage(image);
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


    capture();
}

void VisualServoing::loadMarker1() {
    rw::sensor::Image::Ptr image = rw::loaders::ImageLoader::Factory::load("/home/student/workspace/RoVi-2017/plugin/markers/Marker1.ppm");
    _textureRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
}

void VisualServoing::loadMarker2() {
}

void VisualServoing::loadMarker3() {
    rw::sensor::Image::Ptr image = rw::loaders::ImageLoader::Factory::load("/home/student/workspace/RoVi-2017/plugin/markers/Marker3.ppm");
    _textureRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();

    //load mat for the corner marker
    _img_object = imread("/home/student/workspace/RoVi-2017/plugin/markers/Marker3.ppm",cv::IMREAD_COLOR);
    if(_img_object.empty())
        std::cout << "Failed imread(): image not found" << std::endl;


    //Detect keypoints with surf on the marker. No reason to do this more than once.
    int minHessian = 400;
    _detector = cv::xfeatures2d::SURF::create( minHessian );
    _detector->detectAndCompute( _img_object, cv::Mat() ,_keypoints_object, _descriptors_object );
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


