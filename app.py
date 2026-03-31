from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

from controllers.wiiboard_serial_controller import WiiBoardController
from views.force_view import WiiBoardView, register_signal_handlers


def main():
    controller = WiiBoardController()
    controller.discover_and_connect()

    view = WiiBoardView(controller)
    view.build()
    register_signal_handlers(view)

    anim = FuncAnimation(
        view.fig,
        view.update,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )

    plt.show()
    view.finalize()
    print("All data collection stopped.")


if __name__ == "__main__":
    main()