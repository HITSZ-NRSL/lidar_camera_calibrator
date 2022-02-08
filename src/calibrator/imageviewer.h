#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QMainWindow>
#include <QContextMenuEvent>

#include <memory>

#include <opencv2/opencv.hpp>

namespace Ui {
class ImageViewer;
}

class ImageViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit ImageViewer(QWidget *parent = 0);
    ~ImageViewer();

    void startPickPoints();
    void quitPickPoints();
    void getPickPoitns(std::vector<cv::Point2d>& pts);

public slots:
    void showImage(std::shared_ptr<cv::Mat> img);

protected:
    void contextMenuEvent(QContextMenuEvent *event) override;

private slots:
    void on_actionSave_Image_triggered();

private:
    Ui::ImageViewer *ui;
    std::shared_ptr<cv::Mat> img_;
};

#endif // IMAGEVIEWER_H
