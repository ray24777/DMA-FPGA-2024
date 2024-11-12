# Design Objective
The goal of this design is to develop a neural network acceleration card based on the PCIE interface, aimed at accelerating the digital recognition process of vehicle license plates. By integrating a high-performance neural network accelerator, the card will enable rapid processing of license plate localization, number segmentation, and recognition to meet the host's stringent requirements for real-time performance and accuracy.

# Application Areas
This PCIE acceleration card is primarily intended for use in intelligent transportation systems, parking management systems, and vehicle monitoring systems. By enhancing the speed and accuracy of license plate recognition, these systems can achieve a higher degree of automation and improve user experience.

# Key Technical Features
- **Efficient Neural Network Accelerator**: Utilizes CNN algorithms for license plate number recognition, featuring strong anti-interference capability and accurate recognition performance.
- **Hardware Resource Optimization**: Saves FPGA hardware resources and improves computational efficiency by reusing convolutional layers, simplifying the softmax layer, and employing dynamic fixed-point quantization technology.
- **High-Speed PCIE Interface**: Leverages the PCIE interface for high-speed data transmission between the host and the card, ensuring real-time performance requirements are met.
- **Modular Design**: The system adopts a modular design, which facilitates future maintenance and functional expansion.

# Key Performance Metrics

- **Processing Speed**: Recognition time for each license plate is under 20 ms.
- **Resource Utilization**: FPGA resource usage is controlled within a reasonable range, ensuring stable system operation.
- **Power Consumption**: Overall power consumption is kept low to meet energy-saving requirements.

# Innovations
- **Optimization of Neural Network Accelerator**:  
   The neural network accelerator is specially optimized for license plate recognition tasks, employing a two-stage pipelining approach in the convolutional kernels to increase recognition speed and accuracy.

- **Hardware Resource Reuse**:  
   Through the reuse of convolutional layer designs, FPGA hardware resources are effectively conserved. Given that two convolutional layers in the neural network are of similar size, hardware design allows for maximum reuse, saving hardware resources. Finally, 8-bit dynamic fixed-point quantization is applied to network parameters and intermediate calculations, addressing limitations of FPGA resources and slow floating-point computation speed while ensuring computational precision.

- **Dynamic Fixed-Point Quantization**:  
   This design employs 8-bit dynamic fixed-point quantization for network parameters and intermediate calculation results, solving issues with limited FPGA resources and slow floating-point calculations. In this implementation, 8-bit signed fixed-point numbers are used to quantify data. Since the dynamic range of fixed-point numbers is small and neural networks involve many multiply-accumulate operations, there is a risk of the accelerator's results exceeding the representation range of these fixed-point numbers, which could significantly affect the accuracy of the neural network model. To address this, a dynamic mechanism is introduced. When computing between two vectors, the decimal point position for each element in a vector is consistent, but the decimal positions of the two vectors can differ. After computation, the result is trimmed to 8 bits, and this trimming process is also dynamically configurable, allowing the decimal point position of the result to be set dynamically.

- **Data Alignment in Computation Units**:  
   Since 8-bit dynamic fixed-point numbers are used in this neural network computation, decimal positions may shift after multiplication and addition. In the computation of the next network layer, differences in decimal places between two fixed-point numbers can result in errors when values are summed. Thus, after completing calculations for one layer, the results need to be adjusted to align with the format of the fixed-point numbers used in the next layer’s weight data.
