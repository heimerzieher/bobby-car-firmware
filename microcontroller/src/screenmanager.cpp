#include "screenmanager.h"

#include <vector>

ScreenManager::ScreenManager()
{

}

Screen* ScreenManager::getActiveScreen(void)
{
    return this->activeScreen;
}
