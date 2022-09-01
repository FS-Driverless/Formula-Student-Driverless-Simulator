#include "AirSimGameMode.h"
#include "Misc/FileHelper.h"
#include "IImageWrapperModule.h"
#include "SimHUD/SimHUD.h"
#include "common/Common.hpp"
#include "AirBlueprintLib.h"
#include "common/AirSimSettings.hpp"
#include "Vehicles/Car/SimModeCar.h"
#include "GenericPlatform/GenericPlatformHttp.h"


class AUnrealLog : public msr::airlib::Utils::Logger
{
public:
    virtual void log(int level, const std::string& message) override 
    {
        size_t tab_pos;
        static const std::string delim = ":\t";
        if ((tab_pos = message.find(delim)) != std::string::npos) {
            UAirBlueprintLib::LogMessageString(message.substr(0, tab_pos), 
                message.substr(tab_pos + delim.size(), std::string::npos), LogDebugLevel::Informational);
            
            return; //display only
        }

        if (level == msr::airlib::Utils::kLogLevelError) {
            UE_LOG(LogTemp, Error, TEXT("%s"), *FString(message.c_str()));
        }
        else if (level == msr::airlib::Utils::kLogLevelWarn) {
            UE_LOG(LogTemp, Warning, TEXT("%s"), *FString(message.c_str()));
        }
        else {
            UE_LOG(LogTemp, Log, TEXT("%s"), *FString(message.c_str()));
        }
  
        //also do default logging
        msr::airlib::Utils::Logger::log(level, message);
    }
};

static AUnrealLog GlobalASimLog;

AAirSimGameMode::AAirSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    DefaultPawnClass = nullptr;
    static ConstructorHelpers::FClassFinder<APawn> AirsimSpectatorPawn(TEXT("/AirSim/Blueprints/AirsimSpectatorPawn"));
    SpectatorClass = AirsimSpectatorPawn.Class;
    // DefaultPawnClass = ASpectatorPawn::StaticClass();
    HUDClass = ASimHUD::StaticClass();

    common_utils::Utils::getSetLogger(&GlobalASimLog);

    //module loading is not allowed outside of the main thread, so we load the ImageWrapper module ahead of time.
    static IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(TEXT("ImageWrapper"));
}

void AAirSimGameMode::InitGame(const FString & MapName, const FString & Options, FString & ErrorMessage)
{
    AGameModeBase::InitGame(MapName, Options, ErrorMessage);
    initializeSettings();
}

void AAirSimGameMode::BeginPlay() 
{
    try
    {
        UAirBlueprintLib::OnBeginPlay();
        setUnrealEngineSettings();
    }
    catch (std::exception &ex)
    {
        UAirBlueprintLib::LogMessageString("Error at startup: ", ex.what(), LogDebugLevel::Failure);
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, std::string("Error at startup: ") + ex.what(), "Error");
    }

    Super::BeginPlay();
}

void AAirSimGameMode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    UAirBlueprintLib::OnEndPlay();

    Super::EndPlay(EndPlayReason);
}

void AAirSimGameMode::PreLogin(const FString& Options, const FString& Address, const FUniqueNetIdRepl& UniqueId, FString& ErrorMessage)
{
    FString clientPassword = UGameplayStatics::ParseOption(Options, TEXT("password"));
    std::string serverPassword = msr::airlib::AirSimSettings::singleton().spectator_server_password;
    if(clientPassword != FString(serverPassword.c_str())) {
        // login attamed is blocked if ErrorMessage is set to non-null string
        ErrorMessage = FString(TEXT("Incorrect password"));
    }
}

void AAirSimGameMode::PostLogin(APlayerController * newPlayer) {
    newPlayer->StartSpectatingOnly();
}

void AAirSimGameMode::initializeSettings()
{
    FString settingsText;
    getSettingsText(settingsText);
    msr::airlib::AirSimSettings::initializeSettings(std::string(TCHAR_TO_UTF8(*settingsText)));

    msr::airlib::AirSimSettings::singleton().load();
    for (const auto &warning : msr::airlib::AirSimSettings::singleton().warning_messages)
    {
        UAirBlueprintLib::LogMessageString(warning, "", LogDebugLevel::Failure);
    }
    for (const auto &error : msr::airlib::AirSimSettings::singleton().error_messages)
    {
        UAirBlueprintLib::ShowMessage(EAppMsgType::Ok, error, "settings.json");
    }
}

bool AAirSimGameMode::getSettingsText(FString& settingsTextOutput) {
    return (
        getSettingsTextFromCommandLine(settingsTextOutput)
        ||
        readSettingsTextFromFile(common_utils::FileSystem::combine(std::string(TCHAR_TO_UTF8(*FPaths::LaunchDir())), "settings.json"), settingsTextOutput)
        ||
        readSettingsTextFromFile(common_utils::FileSystem::combine(common_utils::FileSystem::getExecutableFolder(), "settings.json"), settingsTextOutput)
        ||
        readSettingsTextFromFile(common_utils::FileSystem::combine(common_utils::FileSystem::getAppDataFolder(), "settings.json"), settingsTextOutput)
    );
}

bool AAirSimGameMode::readSettingsTextFromFile(std::string settingsFilepath, FString& settingsTextOutput) {
    return readSettingsTextFromFile(FString(settingsFilepath.c_str()), settingsTextOutput);
}


bool AAirSimGameMode::readSettingsTextFromFile(FString settingsFilepath, FString& settingsTextOutput) {
    UAirBlueprintLib::LogMessage("Attempting to load settings from " + settingsFilepath, FString(""), LogDebugLevel::Informational);
    bool found = FPaths::FileExists(settingsFilepath);
    if (found) {
        bool readSuccessful = FFileHelper::LoadFileToString(settingsTextOutput, *settingsFilepath);
        if (readSuccessful) {
            UAirBlueprintLib::LogMessage("Loaded settings from " + settingsFilepath, FString(""), LogDebugLevel::Informational);
        }
        else {
            UAirBlueprintLib::LogMessage("Cannot read file " + settingsFilepath, FString(""), LogDebugLevel::Failure);
            throw std::runtime_error("Cannot read settings file.");
        }
    }

    return found;
}

bool AAirSimGameMode::getSettingsTextFromCommandLine(FString& settingsTextOutput) {
    FString settingsParamValue;

    return (
        FParse::Value(FCommandLine::Get(), TEXT("settings"), settingsParamValue)
        && (
            readSettingsTextFromFile(settingsParamValue.TrimQuotes(), settingsTextOutput)
            || parseSettingsStringFromCommandLine(settingsParamValue, settingsTextOutput)
        )
    );
}

bool AAirSimGameMode::parseSettingsStringFromCommandLine(FString urlEncodedSettings, FString& settingsTextOutput) {
    UAirBlueprintLib::LogMessageString("Attempting to load settings directly from command line: ", std::string(TCHAR_TO_UTF8(*urlEncodedSettings)), LogDebugLevel::Informational);
    
    settingsTextOutput = FGenericPlatformHttp::UrlDecode(urlEncodedSettings);
    UAirBlueprintLib::LogMessageString("Loaded settings directly from command line", std::string(TCHAR_TO_UTF8(*settingsTextOutput)), LogDebugLevel::Informational);
    return true;
}


void AAirSimGameMode::setUnrealEngineSettings()
{
    //TODO: should we only do below on SceneCapture2D components and cameras?
    //avoid motion blur so capture images don't get
    GetWorld()->GetGameViewport()->GetEngineShowFlags()->SetMotionBlur(false);

    //use two different methods to set console var because sometime it doesn't seem to work
    static const auto custom_depth_var = IConsoleManager::Get().FindConsoleVariable(TEXT("r.CustomDepth"));
    custom_depth_var->Set(3);

    //Equivalent to enabling Custom Stencil in Project > Settings > Rendering > Postprocessing
    UKismetSystemLibrary::ExecuteConsoleCommand(GetWorld(), FString("r.CustomDepth 3"));

    //during startup we init stencil IDs to random hash and it takes long time for large environments
    //we get error that GameThread has timed out after 30 sec waiting on render thread
    static const auto render_timeout_var = IConsoleManager::Get().FindConsoleVariable(TEXT("g.TimeoutForBlockOnRenderFence"));
    render_timeout_var->Set(300000);
}
