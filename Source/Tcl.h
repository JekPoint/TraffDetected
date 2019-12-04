#include <sys/types.h>
/**  ласс, позвол€ющий выставить в sharedmemory состо€ни€ каналов детектора пешеходов.
 *
 *
 */
class TclState {
public:
    TclState();
    /**
    * ¬рем€ переключени€ устанавливаетс€ только в случае смены состо€ни€
    *
    */
    void setstate(int index, bool state);
    bool getstate(int index);


    u_int8_t * state;
    int32_t * time;
private:
    int _tclstateid;
    int _tcltimeid;

};