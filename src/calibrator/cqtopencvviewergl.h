#ifndef CQTOPENCVVIEWERGL_H
#define CQTOPENCVVIEWERGL_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_2_0>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <mutex>
#include <memory>

class CQtOpenCVViewerGl : public QOpenGLWidget, protected QOpenGLFunctions_2_0
{
	Q_OBJECT
public:
	explicit CQtOpenCVViewerGl(QWidget *parent = 0);
    void startPickPoints()
    {
        is_pick_points_ = true;
        points_.clear();
    }
    void stopPickPoints()
    {
        is_pick_points_ = false;
        points_.clear();
    }
    void getPickpoints(std::vector<cv::Point2d>& pts)
    {
        pts.clear();
        if(points_.size() >= 4)
        {
            pts = points_;
        }
        stopPickPoints();
        return;
    }

signals:
	void    imageSizeChanged( int outW, int outH ); /// Used to resize the image outside the widget

public slots:
    bool    showImage(const cv::Mat& image, bool isInside = false);        /// Used to set the image to be viewed

protected:
	void 	initializeGL(); /// OpenGL initialization
	void 	paintGL(); /// OpenGL Rendering
	void 	resizeGL(int width, int height);        /// Widget Resize Event

	void        updateScene();
	void        renderImage();

    // pick points
    void    mousePressEvent(QMouseEvent* event);

private:
	QImage      mRenderQtImg;           /// Qt image to be rendered
	QImage      mResizedImg;
	cv::Mat     mOrigImage;             /// original OpenCV image to be shown
    QColor      mBgColor;               /// Background color
    float       mImgRatio;             /// width/height = cols/rows ratio
    int         mRenderWidth;
    int         mRenderHeight;
    int         mRenderPosX;
    int         mRenderPosY;
    std::mutex  drawMutex;

    bool        is_pick_points_;
    cv::Mat     img_;
    std::vector<cv::Point2d> points_;

	void recalculatePosition();
};

#endif // CQTOPENCVVIEWERGL_H
