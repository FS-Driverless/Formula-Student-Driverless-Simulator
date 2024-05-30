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
    
    virtual void InitGame(const FString & MapName, const FString & Options, FString & ErrorMessage) override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void PreLogin(const FString& Options, const FString& Address, const FUniqueNetIdRepl& UniqueId, FString& ErrorMessage) override;
    virtual void PostLogin(APlayerController * NewPlayer) override;

    
    AAirSimGameMode(const FObjectInitializer& ObjectInitializer);

    UPROPERTY() ASimModeBase* simmode;
    
private:
    void initializeSettings();
    bool getSettingsText(FString& settingsTextOutput);
    bool readSettingsTextFromFile(std::string fileName, FString& settingsTextOutput);
    bool readSettingsTextFromFile(FString fileName, FString& settingsTextOutput);
    bool getSettingsTextFromCommandLine(FString& settingsTextOutput);
    bool parseSettingsStringFromCommandLine(FString maybeQuotedString, FString& settingsTextOutput);

    void setUnrealEngineSettings();


};
