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

  private final Address address;
  private WebSocketClient websocket;
  private final Map<String, Consumer<byte[]>> callbacks;
  private final ScheduledExecutorService reconnectExecutor;
  private boolean isReconnecting = false;
  private static final int RECONNECT_DELAY_MS = 5000;

  public Autobahn(Address address) {
    this.address = address;
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
    if (websocket != null && !websocket.isClosed()) {
      isReconnecting = false;
      return;
    }

    System.out.println("Attempting to reconnect to Autobahn server...");
    begin()
        .thenRun(() -> {
          System.out.println("Successfully reconnected to Autobahn server");
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
      try {
        websocket = new WebSocketClient(new URI(address.makeUrl())) {
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
            System.err.println("WebSocket connection closed: " + reason);
            scheduleReconnect();
          }

          @Override
          public void onError(Exception ex) {
            System.err.println("WebSocket error: " + ex.getMessage());
            scheduleReconnect();
          }
        };
        websocket.connect();
      } catch (Exception e) {
        System.err.println("Failed to connect: " + e.getMessage());
        throw new RuntimeException("Failed to connect to WebSocket address");
      }
    });
  }

  public CompletableFuture<Void> publish(String topic, byte[] payload) {
    if (websocket == null || websocket.isClosed()) {
      throw new IllegalStateException(
          "No WebSocket connection available. Call begin() first.");
    }

    return CompletableFuture.runAsync(() -> {
      try {
        PublishMessage message = PublishMessage
            .newBuilder()
            .setMessageType(MessageType.PUBLISH)
            .setTopic(topic)
            .setPayload(ByteString.copyFrom(payload))
            .build();

        websocket.send(message.toByteArray());
      } catch (Exception e) {
        throw new RuntimeException(
            "Failed to publish message: " + e.getMessage());
      }
    });
  }

  public CompletableFuture<Void> subscribe(
      String topic,
      Consumer<byte[]> callback) {
    if (websocket == null || websocket.isClosed()) {
      throw new IllegalStateException(
          "No WebSocket connection available. Call begin() first.");
    }

    return CompletableFuture.runAsync(() -> {
      try {
        callbacks.put(topic, callback);

        TopicMessage message = TopicMessage
            .newBuilder()
            .setMessageType(MessageType.SUBSCRIBE)
            .setTopic(topic)
            .build();

        websocket.send(message.toByteArray());
      } catch (Exception e) {
        throw new RuntimeException("Failed to subscribe: " + e.getMessage());
      }
    });
  }

  public CompletableFuture<Void> unsubscribe(String topic) {
    if (websocket == null || websocket.isClosed()) {
      throw new IllegalStateException(
          "No WebSocket connection available. Call begin() first.");
    }

    return CompletableFuture.runAsync(() -> {
      try {
        callbacks.remove(topic);

        UnsubscribeMessage message = UnsubscribeMessage
            .newBuilder()
            .setMessageType(MessageType.UNSUBSCRIBE)
            .setTopic(topic)
            .build();

        websocket.send(message.toByteArray());
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
      System.err.println("Error in message handler: ");
      e.printStackTrace();
    }
  }
}
