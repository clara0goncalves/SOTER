# SOTER
#Aplicação RTOS e STM32 com Sensor de Pressão e Buzzer usando MQTT

Este projeto tem como principal objetivo o de criar uma aplicação em um Sistema Operativo Tempo Real (RTOS) utilizando um microcontrolador STM32, um sensor de pressão, um buzzer e o protocolo MQTT. Neste projeto, será possível ler dados do sensor de pressão, publicá-los em um tópico MQTT e ativar o buzzer quando um determinado valor é atingido.

## Introdução
Neste trabalho, está apresentado uma aplicação em um Sistema Operativo Tempo Real (RTOS) utilizando um microcontrolador STM32 e um sensor de pressão. A aplicação incluirá também um buzzer e utilizará o protocolo MQTT para comunicação.

## Requisitos de Hardware
- Microcontrolador STM32
- Sensor de pressão
- Buzzer

## Requisitos de Software
1. RTOS (Sistema Operativo Tempo Real)
2. Biblioteca MQTT para STM32
3. STM32CubeIDE ou ambiente de desenvolvimento semelhante

## Configuração do Hardware
1. Conectar o sensor de pressão ao microcontrolador STM32 de acordo com o datasheet do sensor.
2. Conectar o buzzer a uma porta de saída digital no microcontrolador STM32.
3. Configuração do Software
4. Configuração do RTOS no microcontrolador STM32 usando as ferramentas e documentação apropriadas.
5. Instalar a biblioteca MQTT para STM32 e integrá-la no projeto.
6. Efetuar as configurações necessárias para o RTOS, MQTT e sensor de pressão.

## Codificação da Aplicação
1. Criar uma tarefa para ler os dados do sensor de pressão e armazená-los em uma variável.
2. Criar uma tarefa para publicar os dados do sensor num tópico MQTT utilizando a biblioteca MQTT.
3. Criar uma tarefa para monitorar os dados do sensor de pressão e ativar o buzzer quando um determinado limiar é atingido.
4. Integrar todas as tarefas e configurá-las para funcionar no RTOS.

## Testando a Aplicação
1. Compilar e enviar o código para o microcontrolador STM32.
2. Verificação do envio e receção dos dados dos sensores no tópico MQTT.
3. Testar o buzzer de acordo com o valor apresentado no sensor de pressão.
4. Verificação da veracidade dos dados dos sensores enviados pelo broker MQTT. 
