package frc.robot.util.online;

import java.net.URI;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import com.google.protobuf.ByteString;

import proto.autobahn.Message.MessageType;
import proto.autobahn.Message.PublishMessage;
import proto.autobahn.Message.PublishMessageOrBuilder;
import proto.autobahn.Message.TopicMessage;
import proto.autobahn.Message.UnsubscribeMessage;

public class Autobahn {

  private final Address[] addresses;
  private final Map<Address, WebSocketClient> websockets;
  private boolean firstSubscription;
  private final Map<String, Consumer<byte[]>> callbacks;
  private final ScheduledExecutorService reconnectExecutor;
  private boolean isReconnecting = false;
  private static final int RECONNECT_DELAY_MS = 5000;

  public Autobahn(Address[] addresses) {
    this.addresses = addresses;
    this.websockets = new HashMap<>();
    this.firstSubscription = true;
    this.callbacks = new HashMap<>();
    this.reconnectExecutor = Executors.newSingleThreadScheduledExecutor();
  }

  private void scheduleReconnect() {
    if (!isReconnecting) {
      isReconnecting = true;
      reconnectExecutor.schedule(
          this::tryReconnect,
          RECONNECT_DELAY_MS,
          TimeUnit.MILLISECONDS);
    }
  }

  private void tryReconnect() {
    boolean allConnected = true;
    for (Address address : addresses) {
      WebSocketClient ws = websockets.get(address);
      if (ws == null || ws.isClosed()) {
        allConnected = false;
        break;
      }
    }

    if (allConnected) {
      isReconnecting = false;
      return;
    }

    System.out.println("Attempting to reconnect to Autobahn servers...");
    begin()
        .thenRun(() -> {
          System.out.println("Successfully reconnected to all Autobahn servers");
          isReconnecting = false;
          // Resubscribe to all topics
          callbacks.forEach((topic, callback) -> subscribe(topic, callback));
        })
        .exceptionally(ex -> {
          System.err.println("Reconnection attempt failed: " + ex.getMessage());
          scheduleReconnect(); // Schedule another attempt
          return null;
        });
  }

  public CompletableFuture<Void> begin() {
    return CompletableFuture.runAsync(() -> {
      for (Address address : addresses) {
        try {
          WebSocketClient websocket = new WebSocketClient(
              new URI(address.makeUrl())) {
            @Override
            public void onOpen(ServerHandshake handshake) {
            }

            @Override
            public void onMessage(String message) {
            }

            @Override
            public void onMessage(ByteBuffer message) {
              byte[] messageBytes = new byte[message.remaining()];
              message.get(messageBytes);
              handleMessage(messageBytes);
            }

            @Override
            public void onClose(int code, String reason, boolean remote) {
              System.err.println(
                  "WebSocket connection closed for " + address + ": " + reason);
              scheduleReconnect();
            }

            @Override
            public void onError(Exception ex) {
              System.err.println(
                  "WebSocket error for " + address + ": " + ex.getMessage());
              scheduleReconnect();
            }
          };
          websocket.connect();
          websockets.put(address, websocket);
        } catch (Exception e) {
          System.err.println(
              "Failed to connect to " + address + ": " + e.getMessage());
        }
      }

      if (websockets.isEmpty()) {
        throw new RuntimeException(
            "Failed to connect to any WebSocket addresses");
      }
    });
  }

  public CompletableFuture<Void> publishSpecific(Address addr, String topic, byte[] payload) {
    PublishMessage message = PublishMessage
        .newBuilder()
        .setMessageType(MessageType.PUBLISH)
        .setTopic(topic)
        .setPayload(ByteString.copyFrom(payload))
        .build();

    return CompletableFuture.runAsync(() -> {
      try {
        var ws = websockets.get(addr);
        if (ws != null) {
          System.out.println("!!!!");
          ws.send(message.toByteArray());
        }
      } catch (Exception e) {
        System.err.println(
            "Failed to publish to one of the websockets: " + e.getMessage());
      }
    });
  }

  public CompletableFuture<Void> publish(String topic, byte[] payload) {
    if (websockets.isEmpty()) {
      throw new IllegalStateException(
          "No WebSocket connections available. Call begin() first.");
    }

    return CompletableFuture.runAsync(() -> {
      try {
        PublishMessage message = PublishMessage
            .newBuilder()
            .setMessageType(MessageType.PUBLISH)
            .setTopic(topic)
            .setPayload(ByteString.copyFrom(payload))
            .build();

        // Publish to all connected websockets
        websockets
            .values()
            .forEach(ws -> {
              ws.send(message.toByteArray());
            });
      } catch (Exception e) {
        throw new RuntimeException(
            "Failed to publish message: " + e.getMessage());
      }
    });
  }

  public CompletableFuture<Void> subscribe(
      String topic,
      Consumer<byte[]> callback) {
    if (websockets.isEmpty()) {
      throw new IllegalStateException(
          "No WebSocket connections available. Call begin() first.");
    }

    return CompletableFuture.runAsync(() -> {
      try {
        callbacks.put(topic, callback);

        TopicMessage message = TopicMessage
            .newBuilder()
            .setMessageType(MessageType.SUBSCRIBE)
            .setTopic(topic)
            .build();

        websockets.get(addresses[0]).send(message.toByteArray());
        if (firstSubscription) {
          firstSubscription = false;
        }
      } catch (Exception e) {
        throw new RuntimeException("Failed to subscribe: " + e.getMessage());
      }
    });
  }

  public CompletableFuture<Void> unsubscribe(String topic) {
    if (websockets.isEmpty()) {
      throw new IllegalStateException(
          "No WebSocket connections available. Call begin() first.");
    }

    return CompletableFuture.runAsync(() -> {
      try {
        callbacks.remove(topic);

        UnsubscribeMessage message = UnsubscribeMessage
            .newBuilder()
            .setMessageType(MessageType.UNSUBSCRIBE)
            .setTopic(topic)
            .build();

        // Unsubscribe from all connected websockets
        websockets
            .values()
            .forEach(ws -> {
              try {
                ws.send(message.toByteArray());
              } catch (Exception e) {
                System.err.println(
                    "Failed to unsubscribe from one of the websockets: " +
                        e.getMessage());
              }
            });
      } catch (Exception e) {
        throw new RuntimeException("Failed to unsubscribe: " + e.getMessage());
      }
    });
  }

  private void handleMessage(byte[] messageBytes) {
    try {
      PublishMessageOrBuilder messageProto = proto.autobahn.Message.PublishMessage.parseFrom(
          messageBytes);

      if (messageProto.getMessageType() == MessageType.PUBLISH) {
        String topic = messageProto.getTopic();
        if (callbacks.containsKey(topic)) {
          callbacks.get(topic).accept(messageProto.getPayload().toByteArray());
        }
      }
    } catch (Exception e) {
      System.err.println("Error in message handler: " + e.getMessage());
    }
  }
}
