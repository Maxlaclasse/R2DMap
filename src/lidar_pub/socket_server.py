import asyncio
import websockets



async def on_connect(websocket, path):
    print("Client connected")
    await websocket.send("Connection established")

    try:
        while True:
            message = await websocket.recv()
            print(f"Received message from client: {message}")

            # Do some processing on the message if required
            # ...

            # Send back the processed message to the client
            response = f"You sent: {message}"
            await websocket.send(response)

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")

async def main():
    # Change the IP address and port to match your network settings
    uri = "192.168.1.26"
    port = 8085
    async with websockets.serve(on_connect, uri, port):
        print(f"WebSocket server started at {uri}:{port}")
        await asyncio.Future()  # Run forever

asyncio.run(main())