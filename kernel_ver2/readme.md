```
@startuml
skinparam componentStyle rectangle
[Kernel] as kernel
package "Kernel Drivers" {
  component "ens160_driver" as ens
  component "bme280_driver" as bme
  component "mpu9250_driver" as mpu
}
package "User Space" {
  component "sensor_app" as app
  app --> "sensors_thread" 
  app --> "processing_thread"
  app --> "ipc_thread"
  app --> "watchdog_thread"
  app --> "pool_workers"
}
interface "/dev/ens160" as dev_ens
interface "/dev/bme280" as dev_bme
interface "/dev/mpu9250" as dev_mpu
ens -right- dev_ens
bme -right- dev_bme
mpu -right- dev_mpu
sensors_thread --> dev_ens
sensors_thread --> dev_bme
sensors_thread --> dev_mpu
note bottom of app : Uses queues for raw and processed data
kernel --> app : Load modules
@enduml
```
<img width="1575" height="438" alt="image" src="https://github.com/user-attachments/assets/589ccfce-7981-4806-a68c-8a42961342b9" />

```sequenceDiagram
@startuml
participant UserApp as app
participant SensorsThread as sensors
participant ProcessingThread as proc
participant IPCThread as ipc
participant KernelDrivers as drivers
app -> sensors : start thread
app -> proc : start thread
app -> ipc : start thread
loop
  sensors -> drivers : read /dev/*
  drivers --> sensors : data
  sensors -> proc : publish_to_raw_queue
  proc -> proc : fuse data
  proc -> ipc : publish_to_processed_queue
  ipc -> external : send via socket
end
@enduml
```
<img width="785" height="396" alt="image" src="https://github.com/user-attachments/assets/659bf092-55b9-4e34-9b63-5b83bf55fb5f" />

