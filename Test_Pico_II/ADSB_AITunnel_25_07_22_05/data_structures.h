#ifndef DATA_STRUCTURES_HH_
#define DATA_STRUCTURES_HH_

#include <stdint.h>

#include <algorithm>  // For std::copy.

template <class T>
class PFBQueue {
public:
    struct PFBQueueConfig {
        uint16_t buf_len_num_elements = 0;
        T* buffer = nullptr;
        bool overwrite_when_full = false;
    };

    /**
    * Конструктор.
    * ПРИМЕЧАНИЕ: Конструкторы копирования и перемещения не реализованы! Передавайте только по ссылке, чтобы избежать создания ошибки «двойного освобождения»,
    * которая вызвана двумя PFBQueues, разделяющими один и тот же буфер, и обе пытаются освободить его, когда они
    * уничтожаются.
    * @param[in] config_in Определяет длину буфера и указывает на буфер размером buf_len_num_elements+1, если
    * PFBQueue должен работать с предварительно выделенным буфером. Если config_in.buffer оставить как nullptr, буфер будет
    * динамически выделен размером buf_len_num_elements * sizeof(T).
    * @retval Объект PFBQueue.
    */
    PFBQueue(PFBQueueConfig config_in) : config_(config_in), buffer_length_(config_in.buf_len_num_elements)
    {
        if (config_.buffer == nullptr)
        {
            config_.buffer = (T*)malloc(sizeof(T) * buffer_length_);
            buffer_was_dynamically_allocated_ = true;
        }
    }

    /**
    * Деструктор. Освобождает буфер буфера, если он был динамически выделен.
    */
    ~PFBQueue()
    {
        if (buffer_was_dynamically_allocated_ && config_.buffer != nullptr)
        {
            free(config_.buffer);
            config_.buffer = nullptr;  // Prevent double free in case of shallow copy.
        }
    }
    /**
    * Помещает элемент в буфер.
    * @param[in] element Объект для помещения в конец буфера.
    * @retval True в случае успеха, false, если буфер заполнен.
    */
    bool Push(T element)
    {
        uint16_t next_tail = IncrementIndex(tail_);
        if (next_tail == head_)
        {
            if (config_.overwrite_when_full)
            {
                // Overwriting allowed; nudge the head to overwrite the first enqueued element.
                head_++;
            }
            else
            {
                // Overwriting not allowed; this push will result in an error.
                return false;
            }
        }
        config_.buffer[tail_] = element;
        tail_ = next_tail;
        return true;
    }
    /**
    * Извлекает элемент из начала буфера.
    * @param[out] element Ссылка на объект, который будет перезаписан содержимым извлеченного элемента.
    * @retval True в случае успеха, false, если буфер пуст.
    */
    bool Pop(T& element)
    {
        if (head_ == tail_)
        {
            return false;
        }
        element = config_.buffer[head_];
        head_ = IncrementIndex(head_);
        return true;
    }

    /**
    * Возвращает содержимое элемента в буфере, не удаляя его из буфера.
    * @param[out] element Ссылка на объект, который будет перезаписан содержимым прочитанного элемента.
    * @param[in] index Позиция в буфере для прочитанного элемента. По умолчанию 0 (начало буфера).
    * @retval True в случае успеха, false, если буфер пуст или index выходит за пределы.
    */
    bool Peek(T& element, uint16_t index = 0)
    {
        if (index >= Length()) {
            return false;
        }
        element = config_.buffer[IncrementIndex(head_, index)];
        return true;
    }

    /**
   * Возвращает количество элементов в буфере в данный момент.
   * @retval Количество элементов в буфере.
   */
    uint16_t Length()
    {
        if (head_ == tail_)
        {
            return 0;  // Empty.
        }
        else if (head_ > tail_)
        {
            return buffer_length_ - (head_ - tail_);  // Wrapped.
        }
        else
        {
            return tail_ - head_;  // Not wrapped.
        }
    }
    /**
    * Возвращает максимальное количество элементов, которые можно сохранить в очереди. Это на единицу меньше длины
    * буфера.
    * @retval Количество элементов, которые можно сохранить в очереди.
    */
    inline uint16_t MaxNumElements() { return config_.buf_len_num_elements - 1; }

    /**
     * Empty out the buffer by setting the head equal to the tail.
     */
    void Clear() { head_ = tail_; }

private:
    /**
    * Увеличивает и обертывает индекс буфера. Индекс должен быть < 2*(config_.buf_len_num_elements+1)!
    * @param[in] index Значение для увеличения и обертывания.
    * @param[in] increment Значение для увеличения индекса. По умолчанию 1.
    * @retval Увеличенное и обернутое значение.
    */
    uint16_t IncrementIndex(uint16_t index, uint16_t increment = 1)
    {
        index += increment;
        return index >= buffer_length_ ? index - buffer_length_ : index;
    }

    PFBQueueConfig config_;
    bool buffer_was_dynamically_allocated_ = false;
    uint16_t buffer_length_;
    uint16_t head_ = 0;
    uint16_t tail_ = 0;
};

#endif
