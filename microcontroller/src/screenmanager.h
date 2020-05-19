#ifndef SCREENMANAGER_H
#define SCREENMANAGER_H

#include <vector>
#include <bitset>
#include <array>

#include "screen.h"

constexpr std::size_t maxScreens = 8;

extern Screen* currentScreen;

class ScreenManager
{
public:
    ScreenManager();
    template <typename T>
    bool HasScreen() const
    {
        return this->screenBitset[getScreenTypeID<T>()];
    }

    Screen* getActiveScreen(void);

    template <typename T, typename... TArgs>
    T& AddScreen(TArgs&&... mArgs)
    {
        assert(!this->HasScreen<T>());

        T* screen = new T(std::forward<TArgs>(mArgs)...);

        std::unique_ptr<Screen> uniquePtr{ screen };
        screens.emplace_back(std::move(uniquePtr));

        this->screenArray[getScreenTypeID<T>()] = screen;
        this->screenBitset[getScreenTypeID<T>()] = true;

        return *screen;
    }

    template <typename T>
    T& GetScreen() const
    {
        assert(this->HasScreen<T>());
        auto ptr = screenArray[getScreenTypeID<T>()];

        return *reinterpret_cast<T*>(ptr);
    }

    template <typename T>
    T& LoadScreen()
    {
        if(!this->HasScreen<T>())
        {
            this->AddScreen<T>();
            this->GetScreen<T>().create();
        }

        lv_disp_load_scr(this->GetScreen<T>().lv_screen);
        this->activeScreen = &this->GetScreen<T>();
    }

private:
    std::vector<std::unique_ptr<Screen>> screens;

    std::array<Screen*, maxScreens> screenArray;

    std::bitset<maxScreens> screenBitset;

    Screen* activeScreen;
};

#endif // SCREENMANAGER_H
