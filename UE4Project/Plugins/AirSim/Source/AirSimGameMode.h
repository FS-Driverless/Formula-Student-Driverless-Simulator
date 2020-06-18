// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/GameUserSettings.h"
#include "SimMode/SimModeBase.h"
#include "AirSimGameMode.generated.h"

/**
 * 
 */
UCLASS()
class AIRSIM_API AAirSimGameMode : public AGameModeBase
{
public:
	GENERATED_BODY()
    
    virtual void StartPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void PostLogin(APlayerController * NewPlayer) override;

    
    AAirSimGameMode(const FObjectInitializer& ObjectInitializer);

    UPROPERTY() ASimModeBase* simmode;

    
private:
    void initializeSettings();
    bool getSettingsText(std::string& settingsText);
    bool getSettingsTextFromCommandLine(std::string& settingsText);
    bool readSettingsTextFromFile(FString fileName, std::string& settingsText);
    std::string getSimModeFromUser();

    void setUnrealEngineSettings();
    void createSimMode();


};
