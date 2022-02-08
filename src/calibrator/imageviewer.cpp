#include "imageviewer.h"
#include "ui_imageviewer.h"

#include <QMenu>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>

ImageViewer::ImageViewer(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ImageViewer),
    img_(nullptr)
{
    ui->setupUi(this);

    setWindowFlags(Qt::Window
                   | Qt::WindowMinimizeButtonHint
                   | Qt::WindowMaximizeButtonHint);
}

ImageViewer::~ImageViewer()
{
    delete ui;
}


/**
 * @brief show image in standlone window
 * @param img
 */
void ImageViewer::showImage(std::shared_ptr<cv::Mat> img)
{
    ui->image_widget->showImage(*img);
    img_ = img;
}

void ImageViewer::startPickPoints()
{
    ui->image_widget->startPickPoints();
}

void ImageViewer::quitPickPoints()
{
    ui->image_widget->stopPickPoints();
}

void ImageViewer::getPickPoitns(std::vector<cv::Point2d>& pts)
{
    ui->image_widget->getPickpoints(pts);
}

void ImageViewer::contextMenuEvent(QContextMenuEvent *event)
{
    if(img_ == nullptr)
    {
        return;
    }
    QMenu menu(this);
    menu.addAction(ui->actionSave_Image);
    menu.exec(event->globalPos());
}


void ImageViewer::on_actionSave_Image_triggered()
{
    QString fn = QFileDialog::getSaveFileName(this,
                 tr("Save Image"),
                 "",
                 tr("Image(*.png)"));
    qDebug() << fn;
    if(fn.isEmpty())
    {
        return;
    }
    if(!cv::imwrite(fn.toStdString(), *img_))
    {
        QMessageBox::warning(this, tr("Error"), tr("Fail to save image"));
    }
}

