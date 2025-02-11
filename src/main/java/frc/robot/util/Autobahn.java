package frc.robot.util;

import java.net.URI;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Consumer;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

public class Autobahn {

  private enum Flags {
    SUBSCRIBE((byte) 0x01),
    UNSUBSCRIBE((byte) 0x02),
    PUBLISH((byte) 0x03);

    private final byte flag;

    Flags(byte flag) {
      this.flag = flag;
    }

    public byte getFlag() {
      return flag;
    }
  }

  private final String host;
  private final int port;
  private WebSocketClient websocket;
  private final Map<String, Consumer<byte[]>> subscriptions;
  private final ExecutorService executorService;

  public Autobahn(String host, int port) {
    this.host = host;
    this.port = port;
    this.subscriptions = new HashMap<>();
    this.executorService = Executors.newSingleThreadExecutor();
  }

  public CompletableFuture<Void> begin() {
    CompletableFuture<Void> future = new CompletableFuture<>();
    try {
      URI uri = new URI("ws://" + host + ":" + port);
      websocket =
        new WebSocketClient(uri) {
          @Override
          public void onOpen(ServerHandshake handshake) {
            future.complete(null);
          }

          @Override
          public void onMessage(String message) {
            // Not used for binary messages
          }

          @Override
          public void onMessage(ByteBuffer message) {
            handleMessage(message.array());
          }

          @Override
          public void onClose(int code, String reason, boolean remote) {
            // Handle close
          }

          @Override
          public void onError(Exception ex) {
            future.completeExceptionally(ex);
          }
        };
      websocket.connect();
    } catch (Exception e) {
      future.completeExceptionally(e);
    }

    return future;
  }

  public void subscribe(String topic, Consumer<byte[]> callback) {
    subscriptions.put(topic, callback);
    publishInternal(Flags.SUBSCRIBE, topic.getBytes(), new byte[0]);
  }

  public void unsubscribe(String topic) {
    subscriptions.remove(topic);
    publishInternal(Flags.UNSUBSCRIBE, topic.getBytes(), new byte[0]);
  }

  private void handleMessage(byte[] message) {
    try {
      ByteBuffer buffer = ByteBuffer.wrap(message);
      byte flag = buffer.get();
      int topicLength = buffer.getInt();
      byte[] topicBytes = new byte[topicLength];
      buffer.get(topicBytes);
      String topic = new String(topicBytes);
      byte[] payload = new byte[buffer.remaining()];
      buffer.get(payload);

      subscriptions.forEach((subscribedTopic, callback) -> {
        if (topic.startsWith(subscribedTopic)) {
          executorService.execute(() -> callback.accept(payload));
        }
      });
    } catch (Exception e) {
      System.out.println("Error in message handler: " + e.getMessage());
    }
  }

  private void publishInternal(Flags flag, byte[] topic, byte[] message) {
    if (websocket != null && websocket.isOpen()) {
      websocket.send(buildMessage(flag, topic, message));
    }
  }

  public void publish(String topic, byte[] message) {
    publishInternal(Flags.PUBLISH, topic.getBytes(), message);
  }

  private byte[] buildMessage(Flags flag, byte[] topic, byte[] message) {
    int topicLength = topic.length;
    int messageLength = message.length;

    return ByteBuffer
      .allocate(1 + 4 + topicLength + 4 + messageLength)
      .put(flag.getFlag())
      .putInt(topicLength)
      .put(topic)
      .putInt(messageLength)
      .put(message)
      .array();
  }

  public void close() {
    if (websocket != null) {
      websocket.close();
    }

    executorService.shutdown();
  }
}
