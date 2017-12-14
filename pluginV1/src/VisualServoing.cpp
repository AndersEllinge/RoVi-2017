#include "VisualServoing.hpp"

#include <boost/filesystem.hpp>
#include <boost/bind.hpp>
#include <QtWidgets/QVBoxLayout>
#include <QDialog>
#include <QtWidgets/QFileDialog>
#include <rw/kinematics.hpp>

#include "vs.h"
#include "ip.h"
#include "vsMult.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"


#define focal 823.0
#define ez 0.5

std::vector<rw::math::Transform3D<>> loadTransforms(std::string file);
rw::math::Q addVelLimits(rw::math::Q dq, double t, rw::models::Device::Ptr device);


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
    connect(btns[3], SIGNAL(pressed()), this, SLOT(nextMarkerPosMult()));

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

    transformsForMarker = loadTransforms(motionFile);
    transIterator = 0;

    capture();
}

void VisualServoing::loadMarker1() {
    rw::sensor::Image::Ptr image = rw::loaders::ImageLoader::Factory::load("/home/student/workspace/RoVi-2017/plugin/markers/Marker1.ppm");
    _textureRender->setImage(*image);
    getRobWorkStudio()->updateAndRepaint();
    capture();

    std::vector<cv::Point2i> points = ip::marker1Function(_framegrabber->getImage());
    _UVs = {points[0].x, points[0].y};

    std::vector<double> U;
    std::vector<double> V;

    for (int i = 0; i < 3; i++) {
        double u = points[i].x;
        double v = points[i].y;
        U.push_back(u);
        V.push_back(v);
    }

    _UV.push_back(U);
    _UV.push_back(V);

    for (int j = 0; j < _UV[0].size(); j++) {
        rw::common::Log::log().info() << "uvt: " << _UV[0][j] << ", " << _UV[1][j] << std::endl;
    }

}

void VisualServoing::loadMarker2() {
    cv::Point2i p1 = {1, 1};
    cv::Point2i p2 = {249, 249};
    cv::Point2i p3 = {1, 249};
    cv::Point2i p4 = {249, 1};
    std::vector<cv::Point2i> p = {p1, p2, p3, p4};
    _img_object = imread("/home/student/workspace/RoVi-2017/plugin/markers/Marker3.ppm",cv::IMREAD_COLOR);
    if(_img_object.empty())
        std::cout << "Failed imread(): image not found" << std::endl;
    p = ip::toRobotPoints(p, _img_object);

    rw::common::Log::log().info() << "s: " << _img_object.cols << ", " << _img_object.rows << std::endl;
    for (int k = 0; k < p.size(); k++) {
        rw::common::Log::log().info() << "x: " << p[k].x << " y: " << p[k].y << std::endl;
    }
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

void VisualServoing::nextMarkerPos() {
    // Move marker
    rw::kinematics::MovableFrame* mFrame = dynamic_cast<rw::kinematics::MovableFrame*>(_markerFrame);
    mFrame->setTransform(transformsForMarker[transIterator], _state);
    std::cout << "size: " << std::endl << transformsForMarker.size() << std::endl;
    if (transIterator < transformsForMarker.size() - 1) {
        transIterator++;
    }
    getRobWorkStudio()->setState(_state);

    // Get new points from marker
    std::vector<cv::Point2i> points = ip::marker1Function(_framegrabber->getImage());
    std::vector<double> newUV = {points[0].x, points[0].y};

    // Calc du dv
    std::vector<double> dUV = {newUV[0] - _UVs[0], newUV[1] - _UVs[1]};

    // Calc dq
    rw::math::Q dq = vs::calcDqFromUV(_UVs[0], _UVs[1], dUV[0], dUV[1], ez, focal, _cameraFrame, _device, _state);
    dq = addVelLimits(dq, deltaT, _device);
    rw::math::Q newQ = _device->getQ(_state) + dq;
    _device->setQ(newQ, _state);
    getRobWorkStudio()->setState(_state);

    std::cout << "q: " << std::endl << newQ << std::endl;

    // Calc pres
    std::vector<cv::Point2i> pointsr = ip::marker1Function(_framegrabber->getImage());
    std::vector<double> resultUV = {pointsr[0].x, pointsr[0].y};
    std::vector<double> precision = {resultUV[0] - _UVs[0], resultUV[1] - _UVs[1]};

    rw::common::Log::log().info() << "pres: " << precision[0] << ", " << precision[1] << std::endl;

}

void VisualServoing::nextMarkerPosMult() {
    // Move marker
    rw::kinematics::MovableFrame* mFrame = dynamic_cast<rw::kinematics::MovableFrame*>(_markerFrame);
    mFrame->setTransform(transformsForMarker[transIterator], _state);
    if (transIterator < transformsForMarker.size() - 1) {
        transIterator++;
    }
    getRobWorkStudio()->setState(_state);

    // Get new points from marker
    std::vector<cv::Point2i> points = ip::marker1Function(_framegrabber->getImage());
    std::vector<std::vector<double>> newUV;
    std::vector<double> U;
    std::vector<double> V;

    for (int i = 0; i < 3; i++) {
        double u = points[i].x;
        double v = points[i].y;
        U.push_back(u);
        V.push_back(v);
    }
    newUV.push_back(U);
    newUV.push_back(V);

    rw::common::Log::log().info() << std::endl;
    for (int j = 0; j < newUV[0].size(); j++) {
        rw::common::Log::log().info() << "nuv: " << newUV[0][j] << ", " << newUV[1][j] << std::endl;
    }

    // Calc du dv from new points
    std::vector<std::vector<double>> dUV = vsMult::calcDuDv(newUV, _UV);

    rw::common::Log::log().info() << std::endl;
    for (int j = 0; j < dUV[0].size(); j++) {
        rw::common::Log::log().info() << "duv: " << dUV[0][j] << ", " << dUV[1][j] << std::endl;
    }

    // Calc dq
    rw::math::Q dq = vsMult::calcDqFromUV(_UV[0], _UV[1], dUV[0], dUV[1], ez, focal, _cameraFrame, _device, _state);

    // Add dq to device
    dq = addVelLimits(dq, deltaT, _device);
    rw::math::Q newQ = _device->getQ(_state) + dq;
    _device->setQ(newQ, _state);
    getRobWorkStudio()->setState(_state);
    //rw::common::Log::log().info() << std::endl << "dq: " << dq << std::endl;


    // Calc precision
    std::vector<cv::Point2i> pointsp = ip::marker1Function(_framegrabber->getImage());

    for (int k = 0; k < pointsp.size(); k++) {
        //rw::common::Log::log().info() << "x: " << pointsp[k].x << " y: " << pointsp[k].y << std::endl;
    }

    std::vector<std::vector<double>> pres;
    std::vector<double> Up;
    std::vector<double> Vp;

    for (int i = 0; i < 3; i++) {
        double u = pointsp[i].x;
        double v = pointsp[i].y;
        Up.push_back(u);
        Vp.push_back(v);
    }
    pres.push_back(Up);
    pres.push_back(Vp);

    pres = vsMult::calcDuDv(pres, _UV);

    rw::common::Log::log().info() << std::endl;
    for (int j = 0; j < pres[0].size(); j++) {
        rw::common::Log::log().info() << "duvp: " << pres[0][j] << ", " << pres[1][j] << std::endl;
    }
    rw::common::Log::log().info() << std::endl << "----------------------" << std::endl;

}

std::vector<rw::math::Transform3D<>> loadTransforms(std::string file) {
    // Init
    std::vector<rw::math::Transform3D<>> trans;
    std::ifstream f(file);

    // Stream and create transforms
    double x,y,z,r,p,q;
    while(f >> x >> y >> z >> r >> p >> q) {
        rw::math::Transform3D<> tmp(rw::math::Vector3D<>(x,y,z), rw::math::RPY<>(r,p,q).toRotation3D());
        trans.push_back(tmp);
    }

    /*
    // Test
    for (int i = 0; i < trans.size(); i++) {
        std::cout << trans[i] << std::endl;
    }
    */

    return trans;

}

rw::math::Q addVelLimits(rw::math::Q dq, double t, rw::models::Device::Ptr device) {
    rw::math::Q ndq(7);
    rw::math::Q maxQ = device->getVelocityLimits();

    for (int i = 0; i < ndq.size(); ++i) {
        if ((dq[i] / t) >= maxQ[i]) {
            //std::cout << "vel lim: " << maxQ << std::endl;
            //std::cout << "dq / t:  " << dq / t << std::endl;
            ndq[i] = (maxQ[i] * t);
        }
        else {
            ndq[i] = dq[i];
        }
    }

    //std::cout << "ndq: " << ndq << std::endl;
    return ndq;
}



