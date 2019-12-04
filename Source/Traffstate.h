 #ifndef OPENCV_EXPERIMENTS_PEDSTATE_H
 #define OPENCV_EXPERIMENTS_PEDSTATE_H


 #include <sys/types.h>
 /** Класс, позволяющий выставить в sharedmemory состояния каналов детектора пешеходов.
  *
  *
  */
 class PedState {
 public:
     PedState();
     /**
     * Время переключения устанавливается только в случае смены состояния
     *
     */
     void setstate(int index, bool state);
     bool getstate(int index);


     u_int8_t * state;
     int32_t * time;
 private:
     int _pedstateid;
     int _pedtimeid;

 };


 #endif //OPENCV_EXPERIMENTS_PEDSTATE_H
