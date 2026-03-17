package frc.excalib.additional_utilities;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class DoubleKeyMap<K1, K2, V> {
    private final Map<Pair<K1, K2>, V> map = new HashMap<>();

    public void put(K1 key1, K2 key2, V value) {
        map.put(new Pair<>(key1, key2), value);
    }

    public V get(K1 key1, K2 key2) {
        return map.get(new Pair<>(key1, key2));
    }

    public boolean containsKeys(K1 key1, K2 key2) {
        return map.containsKey(new Pair<>(key1, key2));
    }

    private static class Pair<K1, K2> {
        private final K1 first;
        private final K2 second;

        Pair(K1 first, K2 second) {
            this.first = first;
            this.second = second;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Pair<?, ?> pair)) return false;
            return Objects.equals(first, pair.first) && Objects.equals(second, pair.second);
        }

        @Override
        public int hashCode() {
            return Objects.hash(first, second);
        }
    }
}
