# AsthmaTech

**AsthmaTech** is a real-time asthma monitoring and early warning system that leverages environmental and physiological data to detect and predict potential asthma attacks. Designed for portability and continuous use, AsthmaTech empowers users to proactively manage asthma, particularly in high-risk environments.

---

## Table of Contents

- [Overview](#overview)  
- [Technologies Used](#technologies-used)  
  - [Embedded Hardware](#embedded-hardware)  
  - [Machine Learning](#machine-learning)  
  - [Programming Languages](#programming-languages)  
  - [Tools & Frameworks](#tools--frameworks)  
- [Use Cases](#use-cases)  
- [License](#license)  

---

## Overview

AsthmaTech integrates a suite of environmental and physiological sensors to continuously monitor key health and environmental indicators:

- Air Quality  
- Dust Density  
- Temperature  
- Humidity  
- Respiratory Rate  
- Oxygen Saturation

Using a Decision Tree-based machine learning model, the system analyzes real-time sensor data to detect early signs of asthma attacks. Once risk patterns are identified, the system transmits the processed insights to a centralized database, which is connected to a mobile application for intuitive user monitoring and alerts.

---

## Technologies Used

### Embedded Hardware

- **Arduino Uno R4 WiFi**  
- **MQ135** – Air quality sensor  
- **DHT11** – Temperature and humidity sensor  
- **MAX30102** – SpO₂ and heart rate sensor  
- **KY-037** – For detecting respiratory sounds  
- **GP2Y1014AU0F** – Dust density sensor  

### Machine Learning

- **Model**: Decision Tree Classifier  
- **Libraries**: `pandas`, `numpy`, `matplotlib`

### Programming Languages

- **C++** – For embedded systems programming  
- **Python** – For data processing, model training, and visualization

### Tools & Frameworks

- Arduino IDE  
- Jupyter Notebook  
- TinyML

---

## Use Cases

- Pediatric and adult asthma management  
- School and home environment air quality monitoring  
- Early risk detection in urban or industrial areas  
- Personalized health tracking for asthma-prone individuals  

---

## License

This project is licensed under the **MIT License**.
