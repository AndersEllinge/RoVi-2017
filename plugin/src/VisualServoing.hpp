#ifndef VISUALSERVOING_HPP
#define VISUALSERVOING_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/RobWorkStudio.hpp>

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"

//QT
#include <QTimer>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QScrollArea>


class VisualServoing: public rws::RobWorkStudioPlugin
{

Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
#if RWS_USE_QT5
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "VisualServoing.json")
#endif

public:
    VisualServoing();
	virtual ~VisualServoing();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();



private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    QWidget* createMarkerButtons();
    std::vector<cv::Point2i> marker1Function(const rw::sensor::Image& image);
	void marker3Function(const rw::sensor::Image& image);
    int markerMethod;


    rw::kinematics::State _state;
    rw::models::WorkCell* _workcell;

    QTimer* _timer;
    QWidget* _markerButtons;
    QLabel* _camPicture;
    QLabel* _processedPicture;
    QPushButton* _initButton;
    QPushButton* _background;
	cv::Mat _img_object;
    std::vector<cv::KeyPoint> _keypoints_object;
    cv::Mat _descriptors_object;
    cv::Ptr<cv::xfeatures2d::SURF> _detector;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;

	rw::kinematics::Frame* _markerFrame;
	rw::kinematics::Frame* _cameraFrame;
	rw::kinematics::Frame* _base;
	rw::models::Device::Ptr _device;
	std::vector<double> _UV;

private slots:
    void stateChangedListener(const rw::kinematics::State& state);
    void capture();
    void detectMarkers();
    void init();
	void testVisualServoing();
    void loadMarker1();
    void loadMarker2();
    void loadMarker3();
    void loadBackground();
};

#endif
