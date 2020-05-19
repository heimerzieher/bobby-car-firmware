#ifndef SCREEN_H
#define SCREEN_H

#include <memory>
#include <lvgl.h>


class Screen
{
public:
    Screen();
    virtual void create(void);
    virtual void update(void);
    virtual void customDraw(void);
    lv_obj_t* lv_screen;
};

namespace Internal
{
    inline std::size_t getUniqueScreenID() noexcept
    {
        static std::size_t lastID{ 0u };
        return lastID++;
    }
}

template <typename T>
inline std::size_t getScreenTypeID() noexcept
{
    static_assert(std::is_base_of<Screen, T>::value, "T must inherit from Screen");

    static std::size_t typeID{ Internal::getUniqueScreenID() };
    return typeID;
}

#endif // SCREEN_H
