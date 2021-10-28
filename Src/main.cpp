//#include "main.h"

#include "board.h"
//#include "camera.h"
//#include "model.h"

#include "model_settings.h"
#include "stm32h747i_discovery.h"
#include "stm32h747i_discovery_camera.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/models/person_detect_model_data.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

// RGB565 Stuff //
#define COLOR_RGB565_TO_R5(pixel) (((pixel) >> 11) & 0x1F)
#define COLOR_RGB565_TO_R8(pixel)        \
  ({                                     \
    __typeof__(pixel) __pixel = (pixel); \
    __pixel = (__pixel >> 8) & 0xF8;     \
    __pixel | (__pixel >> 5);            \
  })
#define COLOR_RGB565_TO_G6(pixel) (((pixel) >> 5) & 0x3F)
#define COLOR_RGB565_TO_G8(pixel)        \
  ({                                     \
    __typeof__(pixel) __pixel = (pixel); \
    __pixel = (__pixel >> 3) & 0xFC;     \
    __pixel | (__pixel >> 6);            \
  })
#define COLOR_RGB565_TO_B5(pixel) ((pixel)&0x1F)
#define COLOR_RGB565_TO_B8(pixel)        \
  ({                                     \
    __typeof__(pixel) __pixel = (pixel); \
    __pixel = (__pixel << 3) & 0xF8;     \
    __pixel | (__pixel >> 5);            \
  })
#define COLOR_R5_G6_B5_TO_RGB565(r5, g6, b5) (((r5) << 11) | ((g6) << 5) | (b5))
#define COLOR_R8_G8_B8_TO_RGB565(r8, g8, b8) \
  ((((r8)&0xF8) << 8) | (((g8)&0xFC) << 3) | ((b8) >> 3))
#define COLOR_RGB888_TO_Y(r8, g8, b8) \
  ((((r8)*38) + ((g8)*75) + ((b8)*15)) >> 7)  // 0.299R + 0.587G + 0.114B
#define COLOR_RGB565_TO_Y(rgb565)           \
  ({                                        \
    __typeof__(rgb565) __rgb565 = (rgb565); \
    int r = COLOR_RGB565_TO_R8(__rgb565);   \
    int g = COLOR_RGB565_TO_G8(__rgb565);   \
    int b = COLOR_RGB565_TO_B8(__rgb565);   \
    COLOR_RGB888_TO_Y(r, g, b);             \
  })

// Available camera resolutions :
// - CAMERA_R160x120
// - CAMERA_R320x240
// - CAMERA_R480x272
// - CAMERA_R640x480
constexpr uint32_t camera_resolution = CAMERA_R160x120;

tflite::ErrorReporter *error_reporter = nullptr;
const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *input = nullptr;

// An area of memory to use for input, output, and intermediate arrays.
constexpr int kTensorArenaSize = 136 * 1024;
static uint8_t tensor_arena[kTensorArenaSize];

static constexpr std::pair<uint32_t, uint32_t> get_camera_dims(
    const uint32_t camera_resolution);

volatile uint32_t Camera_AllowDma2dCopyCamFrmBuffToLcdFrmBuff = 0;
static void LCD_LL_Convert_RGB565ToARGB8888(void *pSrc, void *pDst,
                                            uint16_t xsize, uint16_t ysize);
static void model_init();

int main(void) {
  Board board;
  board.init();

  // Init camera
  uint32_t camera_status =
      BSP_CAMERA_Init(0, camera_resolution, CAMERA_PF_RGB565);

  if (camera_status != BSP_ERROR_NONE) {
    Board::error_handler();
  }

  HAL_Delay(200);

  // init tflite model
  model_init();

  // Start camera stream
  BSP_CAMERA_Start(0, (uint8_t *)CAMERA_FRAME_BUFFER, CAMERA_MODE_CONTINUOUS);

  while (1) {
  }
}

static void model_init() {
  tflite::InitializeTarget();

  model = tflite::GetModel(g_person_detect_model_data);

  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Board::error_handler();
  }

  // Prepare error resolver
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // tflite::AllOpsResolver resolver;
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroMutableOpResolver<5> micro_op_resolver;
  micro_op_resolver.AddAveragePool2D();
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddDepthwiseConv2D();
  micro_op_resolver.AddReshape();
  micro_op_resolver.AddSoftmax();

  // Build an interpreter to run the model with.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);

  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Board::error_handler();
    return;
  }

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);
}

/**
 * @brief  Button Callback
 * @param  Button Specifies the pin connected EXTI line
 */
void BSP_PB_Callback(Button_TypeDef Button) {
  if (Button == BUTTON_WAKEUP) {
    // Implement logic if needed
  }
}

/**
 * @brief  Camera Frame Event callback.
 */
void BSP_CAMERA_FrameEventCallback(uint32_t Instance) {
  BSP_LED_On(LED_RED);
  BSP_CAMERA_Suspend(0);

  // Convert captured frame to ARGB8888 and copy it to LCD FRAME BUFFER
  const auto [width, height] = get_camera_dims(camera_resolution);
  LCD_LL_Convert_RGB565ToARGB8888((uint32_t *)(CAMERA_FRAME_BUFFER),
                                  (uint32_t *)(LCD_LAYER_0_ADDRESS), width,
                                  height);

  // Prepare buffer for TFlite interpreter
  uint16_t *cam_frame = (uint16_t *)CAMERA_FRAME_BUFFER;
  int8_t *input_buffer = input->data.int8;

  for (size_t row = 0; row < kNumRows; row++) {
    for (size_t col = 0; col < kNumCols; col++) {
      uint16_t pixel565 = cam_frame[row + width * col];
      int pixelgrey = COLOR_RGB565_TO_Y(pixel565);
      input_buffer[row + kNumCols * col] = static_cast<int8_t>(pixelgrey - 127);
    }
  }

  if (kTfLiteOk != interpreter->Invoke()) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed.");
  }

  TfLiteTensor *output = interpreter->output(0);

  int8_t person_score = output->data.uint8[kPersonIndex];
  int8_t no_person_score = output->data.uint8[kNotAPersonIndex];

  char txt[64];
  sprintf(txt, "person: %3d   | not-person: %3d", person_score,
          no_person_score);
  UTIL_LCD_DisplayStringAt(
      0, 10, (uint8_t *)"                                   ", CENTER_MODE);
  UTIL_LCD_DisplayStringAt(0, 10, (uint8_t *)txt, CENTER_MODE);

  BSP_CAMERA_Resume(0);
  BSP_LED_Off(LED_RED);
}

/**
 * @brief  Copy the Captured Picture to the display Frame buffer.
 * @param  pSrc: Pointer to source buffer
 * @param  pDst: Pointer to destination buffer
 * @retval None
 */
static void LCD_LL_Convert_RGB565ToARGB8888(void *pSrc, void *pDst,
                                            uint16_t xsize, uint16_t ysize) {
  uint32_t xPos, yPos, destination;
  uint32_t LcdResX, LcdResY;
  BSP_LCD_GetXSize(0, &LcdResX);
  BSP_LCD_GetYSize(0, &LcdResY);
  /* Configure the DMA2D Mode, Color Mode and output offset */
  hlcd_dma2d.Init.Mode = DMA2D_M2M_PFC;
  hlcd_dma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hlcd_dma2d.Init.OutputOffset = LcdResX - xsize;
  /* DMA2D Callbacks Configuration */
  hlcd_dma2d.XferCpltCallback = NULL;

  /* Foreground Configuration */
  hlcd_dma2d.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
  hlcd_dma2d.LayerCfg[1].InputAlpha = 0xFF;
  hlcd_dma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hlcd_dma2d.LayerCfg[1].InputOffset = 0;

  hlcd_dma2d.Instance = DMA2D;

  /* Calculate the destination transfer address */
  xPos = (LcdResX - xsize) / 2;
  yPos = (LcdResY - ysize) / 2;

  destination = (uint32_t)pDst + (yPos * LcdResX + xPos) * 4;

  /* DMA2D Initialization */
  if (HAL_DMA2D_Init(&hlcd_dma2d) == HAL_OK) {
    if (HAL_DMA2D_ConfigLayer(&hlcd_dma2d, 1) == HAL_OK) {
      if (HAL_DMA2D_Start(&hlcd_dma2d, (uint32_t)pSrc, destination, xsize,
                          ysize) == HAL_OK) {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hlcd_dma2d, 100);
      }
    }
  }
}

/**
 * @brief get camera dimensions from given resolution
 * @param camera_resolution as defined in camera BSP
 * @return pair of [width, height]
 */
static constexpr std::pair<uint32_t, uint32_t> get_camera_dims(
    const uint32_t camera_resolution) {
  switch (camera_resolution) {
    case CAMERA_R160x120:
      return std::make_pair(160, 120);

    case CAMERA_R320x240:
      return std::make_pair(320, 240);

    case CAMERA_R480x272:
      return std::make_pair(480, 272);

    case CAMERA_R640x480:
      return std::make_pair(640, 480);
  }
  return std::make_pair(0, 0);
}
