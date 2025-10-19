# DASCA-Design-Advanced-Sensor-Concurrency-Architecture
```mermaid
graph TD
A[1️⃣ Nền tảng Linux cơ bản] --> B[2️⃣ POSIX & System Programming]
B --> C[3️⃣ Lập trình Driver cảm biến]
C --> D[4️⃣ Concurrency & Synchronization trong Kernel]
D --> E[5️⃣ User-space Concurrency & IPC]
E --> F[6️⃣ Framework & Integration]
F --> G[7️⃣ Real-time & Tracing]
G --> H[8️⃣ System Engineering & Deployment]
H --> I[9️⃣ Ứng dụng mở rộng]
```
```mermaid
graph TD
    A[MPU9250 Sensor] -->|I2C| K[Kernel Driver]
    B[BME280 Sensor] -->|I2C| K
    C[ENS160/AHT21 Sensor] -->|I2C| K
    K -->|ioctl/read/poll| U[User-space Threads]
    U -->|IPC/Queue| P[Processing Thread]
    P -->|Publish| L[Logging/GUI/Cloud]
    W[Watchdog Thread] -->|Monitor| U
    style K fill:#fef9c3,stroke:#f59e0b
    style U fill:#dbeafe,stroke:#2563eb
    style P fill:#bbf7d0,stroke:#16a34a
    style L fill:#fbcfe8,stroke:#db2777
    style W fill:#fee2e2,stroke:#b91c1c
```
