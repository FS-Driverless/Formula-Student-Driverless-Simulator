#include "SimHUDWidget.h"



void USimHUDWidget::toggleHelpVisibility()
{
    setHelpContainerVisibility(!getHelpContainerVisibility());
}

void USimHUDWidget::setOnToggleRecordingHandler(OnToggleRecording handler)
{
    on_toggle_recording_ = handler;
}

void USimHUDWidget::onToggleRecordingButtonClick()
{
    on_toggle_recording_();
}