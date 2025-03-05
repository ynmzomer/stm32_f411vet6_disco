# stm32_f411vet6_disco

# Proje Açıklaması

Bu depo, STM32F411 mikrodenetleyicisi üzerinde register seviyesinde yapılan test uygulamalarını ve sürücü geliştirme çalışmalarını içermektedir. Tüm çalışmalar **STMicroelectronics'in Reference Manual'ı** baz alınarak sıfırdan yazılmıştır.

## Klasör Açıklamaları

### **Register Seviyesinde Test Uygulamaları**
Numaralandırılmış klasörler (`000TestProject`, `002sizeof`, `003Add`, ... vb.), STM32F411 üzerinde **register seviyesiyle** yapılan temel test uygulamalarını içerir. Bu uygulamalar sıfırdan, herhangi bir hazır kütüphane kullanılmadan geliştirilmiştir.

### **Modbus Slave Uygulaması**
- **`modbus_slave`**: Bu klasörde bulunan Modbus Slave uygulaması, **HAL (Hardware Abstraction Layer) kütüphanesi** kullanılarak geliştirilmiştir.
- Modbus RTU protokolüne uygun olarak STM32F411 üzerinde bir slave cihazı çalıştırmayı amaçlamaktadır.

### **STM32F411 Sürücüleri**
- **`stm32f411_drivers`**: Bu klasör, GPIO, UART, SPI ve I2C sürücülerini ve bunların uygulama kodlarını içermektedir.
- Tüm sürücüler, **herhangi bir hazır kütüphane (HAL veya LL) kullanılmadan**, doğrudan **reference manual** incelenerek yazılmıştır.
- Mikrodenetleyicinin donanımına doğrudan erişerek **register bazlı bir sürücü geliştirme** süreci uygulanmıştır.
Aşağıda README dosyanın İngilizce versiyonunu hazırladım:

---

# Project Description

This repository contains **register-level test applications** and **driver development** for the STM32F411 microcontroller. All implementations were written from scratch, strictly following the **STMicroelectronics Reference Manual**.

## Folder Descriptions

### **Register-Level Test Applications**
The numbered folders (`000TestProject`, `002sizeof`, `003Add`, ... etc.) contain **basic test applications** developed at the **register level** on the STM32F411. These applications were built entirely from scratch without using any pre-existing libraries.

### **Modbus Slave Application**
- **`modbus_slave`**: This folder contains a Modbus Slave application developed using the **HAL (Hardware Abstraction Layer) library**.
- The application is designed to implement a Modbus RTU slave device on the STM32F411.

### **STM32F411 Drivers**
- **`stm32f411_drivers`**: This folder contains **custom drivers** for GPIO, UART, SPI, and I2C, along with their corresponding application codes.
- All drivers were developed **entirely from scratch**, without using any pre-built libraries such as HAL or LL.
- The drivers directly interact with the hardware registers, following the **reference manual** for low-level peripheral control.
