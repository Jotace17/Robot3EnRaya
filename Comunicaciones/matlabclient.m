cli = tcpclient('192.168.0.19', 55355); % IP y puerto del servidor ESP32
text=sprintf('hola mi amorcito <3...\n');
cli.write(text);