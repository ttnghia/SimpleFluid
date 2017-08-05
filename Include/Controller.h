//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
//  Copyright (c) 2017 by
//       __      _     _         _____
//    /\ \ \__ _| |__ (_) __ _  /__   \_ __ _   _  ___  _ __   __ _
//   /  \/ / _` | '_ \| |/ _` |   / /\/ '__| | | |/ _ \| '_ \ / _` |
//  / /\  / (_| | | | | | (_| |  / /  | |  | |_| | (_) | | | | (_| |
//  \_\ \/ \__, |_| |_|_|\__,_|  \/   |_|   \__,_|\___/|_| |_|\__, |
//         |___/                                              |___/
//
//  <nghiatruong.vn@gmail.com>
//  All rights reserved.
//
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

#pragma once

#include <QWidget>
#include <QtWidgets>

#include <QtAppHelpers/BrowsePathWidget.h>
#include <QtAppHelpers/MaterialSelector.h>
#include <QtAppHelpers/EnhancedComboBox.h>
#include <QtAppHelpers/EnhancedSlider.h>
#include <QtAppHelpers/PointLightEditor.h>

#include "Common.h"

//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class Controller : public QWidget
{
    Q_OBJECT
    friend class MainWindow;
public:
    explicit Controller(QWidget* parent) :
        QWidget(parent)
    {
        setupGUI();
    }

    void updateSimParams(const std::shared_ptr<SimulationParameters>& simParams);
    void disableParameters(bool disable);

public slots:
    void loadTextures();

private:
    void setupGUI();
    void setupSimulationControllers(QVBoxLayout* ctrLayout);
    void setupTextureControllers(QBoxLayout* ctrLayout);
    void setupColorControllers(QBoxLayout* ctrLayout);

    ////////////////////////////////////////////////////////////////////////////////
    QComboBox* m_cbNumThreads;
    QComboBox* m_cbSimulationScene;

    QLineEdit* m_txtPressureStiffness;
    QLineEdit* m_txtViscosity;
    QLineEdit* m_txtRestitution;
    QLineEdit* m_txtStopTime;

    QRadioButton* m_rbAttrPressureYes;
    QRadioButton* m_rbAttrPressureNo;

    QSignalMapper* m_smParticleColorMode;

    EnhancedComboBox* m_cbSkyTexture;
    EnhancedComboBox* m_cbFloorTexture;
    EnhancedSlider*   m_sldFloorSize;
    EnhancedSlider*   m_sldFloorExposure;
    MaterialSelector* m_msParticleMaterial;

    QPushButton* m_btnStartStopSimulation;
    QPushButton* m_btnResetSimulation;

    PointLightEditor* m_LightEditor;
};