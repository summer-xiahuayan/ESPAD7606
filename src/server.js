const net = require('net');
const fs = require('fs');
const path = require('path');
const WebSocket = require('ws');
const os = require('os');

// 配置参数
const TCP_PORT = 3000; // ESP32连接的端口
const WS_PORT = 1234;  // 网页连接的WebSocket端口
const DATA_DIR = path.join(__dirname, 'data');
const CONFIG = {
    CSV_DELIMITER: ',',
    ENABLE_BOM: true
};

// 确保数据目录存在
if (!fs.existsSync(DATA_DIR)) {
    fs.mkdirSync(DATA_DIR);
}

// 存储当前连接的ESP32客户端和日志文件
let espClient = null;
let currentLogStream = null;
let currentLogFile = null;
let wsClients = new Set();

// 生成带毫秒的本地时间戳
function generateLocalTimestampWithMs() {
    const now = new Date();
    const utcTime = new Date(now.getTime() + now.getTimezoneOffset() * 60000);
    const localTime = new Date(utcTime.getTime() + 8 * 60 * 60 * 1000);

    return [
        localTime.getFullYear(),
        String(localTime.getMonth() + 1).padStart(2, '0'),
        String(localTime.getDate()).padStart(2, '0')
    ].join('-') + 'T' + [
        String(localTime.getHours()).padStart(2, '0'),
        String(localTime.getMinutes()).padStart(2, '0'),
        String(localTime.getSeconds()).padStart(2, '0'),
        String(localTime.getMilliseconds()).padStart(3, '0')
    ].join('-');
}

// 获取系统默认编码
function getSystemEncoding() {
    const platform = os.platform();
    return platform === 'win32' ? 'gbk' : 'utf8';
}

// 创建TCP服务器
const tcpServer = net.createServer((socket) => {
    console.log('ESP32客户端已连接');
    if (espClient) espClient.destroy();
    espClient = socket;
    broadcastStatus('ESP_CONNECTED');

    socket.on('data', (data) => {
        try {
            const dataString = data.toString().trim();
            const values = dataString.split(',').map(Number);

            if (values.length === 8) {
                const timestamp = generateExcelFriendlyTimestamp();
                const logLine = `${timestamp}${CONFIG.CSV_DELIMITER}${values.join(CONFIG.CSV_DELIMITER)}\n`;
                console.log(logLine);

                if (currentLogStream) {
                    const systemEncoding = getSystemEncoding();
                    const encodedData = systemEncoding === 'utf8' ? logLine : iconv.encode(logLine, 'gbk');
                    currentLogStream.write(encodedData);
                }

                broadcastData(values);
            } else {
                console.log(`收到格式不正确的数据: ${dataString}`);
            }
        } catch (error) {
            console.error(`处理数据时出错: ${error.message}`);
        }
    });

    socket.on('close', () => {
        console.log('ESP32客户端已断开连接');
        if (espClient === socket) {
            espClient = null;
            broadcastStatus('WAITING_FOR_ESP');
        }
    });

    socket.on('error', (error) => {
        console.error(`ESP32客户端错误: ${error.message}`);
        if (espClient === socket) {
            espClient = null;
            broadcastStatus('WAITING_FOR_ESP');
        }
    });
});

// 创建WebSocket服务器
const wss = new WebSocket.Server({ port: WS_PORT });
const iconv = require('iconv-lite');

wss.on('connection', (ws) => {
    console.log('网页客户端已连接');
    wsClients.add(ws);

    ws.send(`STATUS:${espClient ? 'ESP_CONNECTED' : 'WAITING_FOR_ESP'}`);

    ws.on('message', (message) => {
        const msg = message.toString().trim();

        if (msg === 'GET_STATUS') {
            ws.send(`STATUS:${espClient ? 'ESP_CONNECTED' : 'WAITING_FOR_ESP'}`);
        } else if (msg === 'START_LOGGING') {
            startLogging();
        } else if (msg === 'STOP_LOGGING') {
            stopLogging();
        } else if (msg === 'STOP_SERVER') {
            stopServer();
        } else if (msg.startsWith('DOWNLOAD_CSV:')) {
            const filename = msg.substring(13);
            console.log(filename);
            downloadCsv(ws, filename);
        }
    });

    ws.on('close', () => {
        console.log('网页客户端已断开连接');
        wsClients.delete(ws);
    });
});

// 广播数据
function broadcastData(data) {
    const message = `DATA:${data.join(',')}`;
    wsClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}

// 广播状态
function broadcastStatus(status) {
    const message = `STATUS:${status}`;
    wsClients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(message);
        }
    });
}

// 开始记录数据
function startLogging() {
    if (currentLogStream) {
        currentLogStream.end();
    }

    const timestamp = generateLocalTimestampWithMs();
    const fileName = path.join(DATA_DIR, `ad7606_${timestamp}.csv`);
    currentLogFile = path.basename(fileName);

    const systemEncoding = getSystemEncoding();
    currentLogStream = fs.createWriteStream(fileName, {
        flags: 'a',
        encoding: systemEncoding === 'utf8' ? 'utf8' : 'binary'
    });

    if (systemEncoding === 'utf8' && CONFIG.ENABLE_BOM) {
        currentLogStream.write('\ufeff');
    }

    const header = `时间${CONFIG.CSV_DELIMITER}通道1${CONFIG.CSV_DELIMITER}通道2${CONFIG.CSV_DELIMITER}通道3${CONFIG.CSV_DELIMITER}通道4${CONFIG.CSV_DELIMITER}通道5${CONFIG.CSV_DELIMITER}通道6${CONFIG.CSV_DELIMITER}通道7${CONFIG.CSV_DELIMITER}通道8\n`;
    const encodedHeader = systemEncoding === 'utf8' ? header : iconv.encode(header, 'gbk');
    currentLogStream.write(encodedHeader);

    console.log(`开始记录数据到 ${fileName}`);
    broadcastStatus(`LOGGING_STARTED:${currentLogFile}`);
}

// 停止记录数据
function stopLogging() {
    if (currentLogStream) {
        currentLogStream.end();
        currentLogStream = null;
        currentLogFile = null;
        console.log('停止记录数据');
        broadcastStatus('LOGGING_STOPPED');
    }
}

// 下载CSV文件
function downloadCsv(ws, filename) {
    const filePath = path.join(DATA_DIR, filename);
    if (!fs.existsSync(filePath)) {
        ws.send('STATUS:NO_LOG_FILE');
        return;
    }

    try {
        const systemEncoding = getSystemEncoding();
        const content = systemEncoding === 'utf8' ?
            fs.readFileSync(filePath, 'utf8') :
            iconv.decode(fs.readFileSync(filePath), 'gbk');

        ws.send(`CSV_DATA:${content}`);
    } catch (error) {
        console.error(`读取CSV文件出错: ${error.message}`);
        ws.send('STATUS:ERROR_READING_CSV');
    }
}

// 停止服务器
function stopServer() {
    stopLogging();
    if (espClient) {
        espClient.destroy();
        espClient = null;
    }

    tcpServer.close(() => console.log('TCP服务器已关闭'));
    wss.clients.forEach(client => client.close());
    wss.close(() => console.log('WebSocket服务器已关闭'));
}

// 启动TCP服务器
tcpServer.listen(TCP_PORT, () => {
    console.log(`TCP服务器运行在 ${TCP_PORT} 端口，等待ESP32连接`);
    console.log(`WebSocket服务器运行在 ${WS_PORT} 端口，等待网页连接`);
    console.log(`数据将存储在 ${DATA_DIR} 目录`);
});

// 处理错误
tcpServer.on('error', (error) => {
    if (error.code === 'EADDRINUSE') {
        console.error(`端口 ${TCP_PORT} 已被占用`);
        process.exit(1);
    }
    console.error(`TCP服务器错误: ${error.message}`);
});

// 处理程序退出
process.on('SIGINT', () => {
    stopServer();
    process.exit(0);
});

// 生成Excel兼容的时间戳
function generateExcelFriendlyTimestamp() {
    const now = new Date();
    const utcTime = new Date(now.getTime() + now.getTimezoneOffset() * 60000);
    const localTime = new Date(utcTime.getTime() + 8 * 60 * 60 * 1000);

    return [
        localTime.getFullYear(),
        String(localTime.getMonth() + 1).padStart(2, '0'),
        String(localTime.getDate()).padStart(2, '0')
    ].join('-') + ' ' + [
        String(localTime.getHours()).padStart(2, '0'),
        String(localTime.getMinutes()).padStart(2, '0'),
        String(localTime.getSeconds()).padStart(2, '0'),
        String(localTime.getMilliseconds()).padStart(3, '0')
    ].join(':');
}