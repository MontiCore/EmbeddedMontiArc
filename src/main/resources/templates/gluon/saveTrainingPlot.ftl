            logging.getLogger('matplotlib').setLevel(logging.WARNING)
            import matplotlib.pyplot as plt
            from matplotlib.ticker import MaxNLocator
            import pandas as pd

            metrics_df = pd.read_csv(training_metrics_log_path)
            batch_loss_df = pd.read_csv(batch_loss_log_path)

            plt.figure(figsize=(10, 5))
            plt.plot(metrics_df['Epoch'], metrics_df['Train Metric Score'], label='Train Metric Score')
            plt.plot(metrics_df['Epoch'], metrics_df['Test Metric Score'], label='Test Metric Score')
            plt.xlabel('Epoch')
            plt.ylabel('Metric Score')
            plt.title('Training and Test Metric Scores Over Epochs')
            plt.legend()
            plt.savefig(self.parameter_path(0, dataset) / 'training_metrics_log.svg', format='svg')

            plt.figure(figsize=(10, 5))
            plt.plot(metrics_df['Epoch'], metrics_df['Train Loss'], label='Train Loss')
            plt.plot(metrics_df['Epoch'], metrics_df['Test Loss'], label='Test Loss')
            plt.xlabel('Epoch')
            plt.ylabel('Loss')
            plt.title('Training and Test Loss Over Epochs')
            plt.legend()
            plt.savefig(self.parameter_path(0, dataset) / 'training_test_loss.svg', format='svg')

            plt.figure(figsize=(10, 5))
            ax1 = plt.gca()
            ax1.plot(metrics_df['Epoch'], metrics_df['Train Metric Score'], label='Train Metric Score', color='tab:blue')
            ax1.plot(metrics_df['Epoch'], metrics_df['Test Metric Score'], label='Test Metric Score', color='tab:cyan')
            ax1.set_xlabel('Epoch')
            ax1.set_ylabel('Metric Score', color='tab:blue')
            ax1.tick_params(axis='y', labelcolor='tab:blue')
            ax1.xaxis.set_major_locator(MaxNLocator(integer=True))
            ax1.set_title('Metrics and Loss Over Epochs')
            ax1.legend(loc='center left')
            ax2 = ax1.twinx()
            ax2.plot(metrics_df['Epoch'], metrics_df['Train Loss'], label='Train Loss', color='tab:red', linestyle='--')
            ax2.plot(metrics_df['Epoch'], metrics_df['Test Loss'], label='Test Loss', color='tab:orange', linestyle='--')
            ax2.set_ylabel('Loss', color='tab:red')
            ax2.tick_params(axis='y', labelcolor='tab:red')
            ax2.legend(loc='center right')
            plt.savefig(self.parameter_path(0, dataset) / 'combined_metrics_and_loss.svg', format='svg')
