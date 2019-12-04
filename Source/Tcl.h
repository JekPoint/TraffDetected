#include <sys/types.h>
/** �����, ����������� ��������� � sharedmemory ��������� ������� ��������� ���������.
 *
 *
 */
class TclState {
public:
    TclState();
    /**
    * ����� ������������ ��������������� ������ � ������ ����� ���������
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