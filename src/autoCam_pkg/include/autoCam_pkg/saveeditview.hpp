#ifndef SAVEEDITVIEW_H
#define SAVEEDITVIEW_H

#include <QDialog>
#include "ui_saveeditview.h"

namespace autoCam_pkg {

class SaveEditView : public QDialog {
Q_OBJECT

public:
    SaveEditView(QWidget *parent = 0);
    ~SaveEditView();

    static int count; // declaration

private Q_SLOTS:
    void on_exit_pushButton_clicked();

    void on_SaveToView_pushButton_clicked();

    void on_editView1_pushButton_clicked();

    void on_editView2_pushButton_clicked();

    void on_editView3_pushButton_clicked();

    void on_editView4_pushButton_clicked();

    void on_editView5_pushButton_clicked();

    void on_JA1_horizontalSlider_valueChanged(int value);

    void on_JA2_horizontalSlider_valueChanged(float value);

    void on_JA3_horizontalSlider_valueChanged(float value);

    void on_JA4_horizontalSlider_valueChanged(float value);

    void on_JA5_horizontalSlider_valueChanged(float value);

    void on_JA6_horizontalSlider_valueChanged(float value);

    void on_JA7_horizontalSlider_valueChanged(float value);

    void on_saveNew_pushButton_clicked();


private:
    Ui::SaveEditView *ui;
};

}

#endif // SAVEEDITVIEW_H
