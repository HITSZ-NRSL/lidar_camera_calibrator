#include "cqtopencvviewergl.h"
#include <QMouseEvent>
#include <QDebug>

CQtOpenCVViewerGl::CQtOpenCVViewerGl(QWidget *parent) :
    QOpenGLWidget(parent),
    is_pick_points_(false)
{
    points_.reserve(4);

	mBgColor = QColor::fromRgb(150, 150, 150);
}

void CQtOpenCVViewerGl::initializeGL()
{
	makeCurrent();
	initializeOpenGLFunctions();

	float r = ((float)mBgColor.darker().red())/255.0f;
	float g = ((float)mBgColor.darker().green())/255.0f;
	float b = ((float)mBgColor.darker().blue())/255.0f;
	glClearColor(r,g,b,1.0f);
}

void CQtOpenCVViewerGl::resizeGL(int width, int height)
{
	makeCurrent();
	glViewport(0, 0, (GLint)width, (GLint)height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(0, width, -height, 0, 0, 1);

	glMatrixMode(GL_MODELVIEW);

	recalculatePosition();

	emit imageSizeChanged(mRenderWidth, mRenderHeight);

	updateScene();
}

void CQtOpenCVViewerGl::updateScene()
{
	if (this->isVisible()) update();
}

void CQtOpenCVViewerGl::paintGL()
{
	makeCurrent();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderImage();
}

void CQtOpenCVViewerGl::renderImage()
{

	drawMutex.lock();
	makeCurrent();

	glClear(GL_COLOR_BUFFER_BIT);

	if (!mRenderQtImg.isNull())
	{
		glLoadIdentity();

		glPushMatrix();
		{
			if (mResizedImg.width() <= 0)
			{
				if (mRenderWidth == mRenderQtImg.width() && mRenderHeight == mRenderQtImg.height())
					mResizedImg = mRenderQtImg;
				else
					mResizedImg = mRenderQtImg.scaled(QSize(mRenderWidth, mRenderHeight),
                                                      Qt::IgnoreAspectRatio,
                                                      Qt::SmoothTransformation);
			}

			// ---> Centering image in draw area

			glRasterPos2i(mRenderPosX, mRenderPosY);

			glPixelZoom(1, -1);

			glDrawPixels(mResizedImg.width(), mResizedImg.height(), GL_RGBA, GL_UNSIGNED_BYTE, mResizedImg.bits());

		}
		glPopMatrix();

		// end
		glFlush();
	}

	drawMutex.unlock();
}

void CQtOpenCVViewerGl::recalculatePosition()
{
	mImgRatio = (float)mOrigImage.cols/(float)mOrigImage.rows;

	mRenderWidth = this->size().width();
	mRenderHeight = floor(mRenderWidth / mImgRatio);

	if (mRenderHeight > this->size().height())
	{
		mRenderHeight = this->size().height();
		mRenderWidth = floor(mRenderHeight * mImgRatio);
	}

	mRenderPosX = floor((this->size().width() - mRenderWidth) / 2);
	mRenderPosY = -floor((this->size().height() - mRenderHeight) / 2);

	mResizedImg = QImage();
}

bool CQtOpenCVViewerGl::showImage(const cv::Mat& image, bool isInside)
{
    // for pick poitns
    if(!isInside)
    {
        image.copyTo(img_);
    }

	drawMutex.lock();
	if (image.channels() == 3)
		cvtColor(image, mOrigImage, cv::COLOR_BGR2RGBA);
	else if (image.channels() == 1)
		cvtColor(image, mOrigImage, cv::COLOR_GRAY2RGBA);
	else if (image.channels() == 4)
		mOrigImage = image;
	else return false;

	mRenderQtImg = QImage((const unsigned char*)(mOrigImage.data),
                          mOrigImage.cols, mOrigImage.rows,
                          mOrigImage.step1(), QImage::Format_RGB32);

	recalculatePosition();

	updateScene();
	drawMutex.unlock();
	return true;
}

void CQtOpenCVViewerGl::mousePressEvent(QMouseEvent *event)
{
    if(!is_pick_points_ || event->button() != Qt::LeftButton)
    {
        return;
    }

    int off_x = 0, off_y = 0;
    int img_x = 0, img_y = 0;
    if(width()/height() > mImgRatio )
    {
        img_y = height();
        off_y = 0;
        img_x = floor(height()*mImgRatio);
        off_x = floor((width() - img_x)/2);
    }
    else
    {
        img_x = width();
        off_x = 0;
        img_y = floor(width()/mImgRatio);
        off_y = floor((height() - img_y)/2);
    }

    double x = static_cast<double>(event->x() - off_x)/img_x*img_.cols;
    double y = static_cast<double>(event->y() - off_y)/img_y*img_.rows;

    points_.emplace_back(x,y);
    cv::Mat img;
    img_.copyTo(img);
    for(const auto& p: points_)
    {
        cv::circle(img, p, double(img.rows) /150, cv::Scalar(0,0,255),img.rows/150 + 3);
    }
    showImage(img, true);
}
