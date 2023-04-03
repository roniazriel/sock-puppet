import time
from statistics import mean
import pandas as pd
import numpy as np
import seaborn as sns
# import optuna
import xgboost as xgb
# from _plotly_utils.colors import n_colors
from sklearn.model_selection import train_test_split
from sklearn.metrics import f1_score, accuracy_score, recall_score, precision_score, \
    confusion_matrix, roc_auc_score
import io
# import plotly.express as px
# import plotly.io as pio
# from plotly.subplots import make_subplots
# import plotly.graph_objects as go
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
import joblib
from sklearn.multiclass import OneVsRestClassifier

'''Ignore Warnings'''
pd.options.mode.chained_assignment = None
import warnings

warnings.filterwarnings("ignore", category=UserWarning)


def split_data(X, y, seed=456, val=True):
    """ Split to datasets - A certain arm will appear only in one set, the prediction will be for all ten points """

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=seed)
    if val:
        X_train, X_val, y_train, y_val = train_test_split(X_train, y_train, test_size=0.25, random_state=seed)
        print("Datasets Shapes: ")
        print("X train shape", X_train.shape)
        print("y train shape", y_train.shape)
        print("X val shape", X_val.shape)
        print("y val shape", y_val.shape)
        print("X test shape", X_test.shape)
        print("y test shape", y_test.shape)
        return X_train, X_test, X_val, y_train, y_test, y_val
    else:
        print("Datasets Shapes: ")
        print("X train shape", X_train.shape)
        print("y train shape", y_train.shape)
        print("X test shape", X_test.shape)
        print("y test shape", y_test.shape)
        return X_train, X_test, y_train, y_test

    # plot_class_hist(y_train=y_train, y_test=y_test, plot_path=plot_path,method=method, cv=cv)


def preprocessing(file_name):
    df1 = pd.read_csv(file_name)
    df1.drop(columns=['Unnamed: 0', 'Move duration', 'Manipulability - mu',
                      'Mid joint proximity', 'Max Mid joint proximity', 'Sum Mid joint proximity- all joints',
                      'Arm_ID', 'Joint1 type', 'Joint1 axis', 'Link1 length'], inplace=True)

    df1['GC1'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 1)), 1, 0)
    df1['GC2'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 2)), 1, 0)
    df1['GC3'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 3)), 1, 0)
    df1['GC4'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 4)), 1, 0)
    df1['GC5'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 5)), 1, 0)
    df1['GC6'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 6)), 1, 0)
    df1['GC7'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 7)), 1, 0)
    df1['GC8'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 8)), 1, 0)
    df1['GC9'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 9)), 1, 0)
    df1['GC10'] = np.where(((df1['Success'] > 0) & (df1['Point number'] == 10)), 1, 0)
    df1['Total_GC'] = (df1[['GC1', 'GC2', 'GC3', 'GC4', 'GC5', 'GC6', 'GC7', 'GC8', 'GC9', 'GC10']] == 1).sum(axis=1)

    df1.drop(columns=['Point number', 'Success'], inplace=True)

    df2 = df1.groupby(['Link2 length', 'Link3 length', 'Link4 length', 'Link5 length', 'Link6 length',
                       'Joint2 type', 'Joint2 axis', 'Joint3 type', 'Joint3 axis', 'Joint4 type', 'Joint4 axis',
                       'Joint5 type', 'Joint5 axis', 'Joint6 type', 'Joint6 axis']).sum().reset_index()
    df2 = pd.get_dummies(df2
                         , columns=['Joint2 type', 'Joint2 axis', 'Joint3 type', 'Joint3 axis', 'Joint4 type',
                                    'Joint4 axis',
                                    'Joint5 type', 'Joint5 axis', 'Joint6 type', 'Joint6 axis']
                         , drop_first=True
                         )

    names = ["GC" + str(i) for i in range(1, 11)]
    names.append("Total_GC")
    y = df2[df2.columns.intersection(names)]
    X = df2.drop(columns=df2.columns.intersection(names))
    return X, y


def objective(trial, X_train, y_train, X_val, y_val):
    """Define the objective function"""
    params = {
        'max_depth': trial.suggest_int('max_depth', 1, 9),
        'learning_rate': trial.suggest_float('learning_rate', 0.01, 1.0),
        'n_estimators': trial.suggest_int('n_estimators', 50, 500),
        'gamma': trial.suggest_float('gamma', 1e-8, 1.0),
        'reg_alpha': trial.suggest_float('reg_alpha', 1e-8, 1.0),
        'reg_lambda': trial.suggest_float('reg_lambda', 1e-8, 1.0),
        'scale_pos_weight': trial.suggest_float('scale_pos_weight', 1,
                                                y_train.value_counts()[0] / y_train.value_counts()[1]),
        'max_delta_step': 3,
        'eval_metric': 'logloss'
    }
    # Fit the model
    optuna_model = xgb.XGBClassifier(**params)
    optuna_model.fit(X_train, y_train)

    # Make predictions
    y_pred = optuna_model.predict(X_val)
    y_prob = optuna_model.predict_proba(X_val)[:, 1]

    # Evaluate predictions
    cm = confusion_matrix(y_val, y_pred)
    print("Confusion matrix: ", cm)

    TN = cm[0][0]
    FN = cm[0][1]
    FP = cm[1][0]
    TP = cm[1][1]
    ACC = (TN + TP) / (TN + FN + TP + FP)
    Precision = TP / (TP + FP)
    Recall = TP / (TP + FN)
    F1 = 2 * ((Precision * Recall) / (Precision + Recall))
    print("Self-calculated  Results:")
    print("Test Accuracy : %.4g" % ACC)
    print("Precision : %.4g" % Precision)
    print("Recall : %.4g" % Recall)
    print("F1 score : %.4g" % F1)
    print("ROC-AUC: %.4g" % roc_auc_score(y_val, y_prob))

    return F1


def hyper_parameter_tuning(X_train, y_train, X_val, y_val):
    # Wrap the objective inside a lambda and call objective inside it
    func = lambda trial: objective(trial, X_train, y_train, X_val, y_val)
    study = optuna.create_study(direction="maximize", study_name='XGB optimization')
    study.optimize(func, timeout=6 * 60)  # 6 hours

    print('Number of finished trials: {}'.format(len(study.trials)))
    print('Best trial:')
    trial = study.best_trial

    print('  Value: {}'.format(trial.value))
    print('  Params: ')

    for key, value in trial.params.items():
        print('    {}: {}'.format(key, value))

    xgb_params = trial.params
    return xgb_params


def train_per_label(X_train, y_train, X_test, y_test, weights, label):
    print("Train and optimize model for: ", label)
    if weights:
        best_params = {'scale_pos_weight': y_train[label].value_counts()[0] / y_train[label].value_counts()[1]}
    else:
        best_params = hyper_parameter_tuning(X_train, y_train[[label]], X_test, y_test[[label]])

    print("Best params: ", best_params)
    classifier = xgb.XGBClassifier(**best_params)
    return classifier, best_params


def classify_label(X_train, y_train, X_test, classifer, label):
    print("Predicting")
    classifer.fit(X_train, y_train[[label]])
    preds = classifer.predict(X_test)
    return preds


def print_confusion_matrix(confusion_matrix, axes, class_label, class_names, fontsize=14):
    df_cm = pd.DataFrame(
        confusion_matrix, index=class_names, columns=class_names,
    )

    try:
        heatmap = sns.heatmap(df_cm, annot=True, fmt='.0%', cbar=False, ax=axes)
    except ValueError:
        raise ValueError("Confusion matrix values must be integers.")
    heatmap.yaxis.set_ticklabels(heatmap.yaxis.get_ticklabels(), rotation=0, ha='right', fontsize=fontsize)
    heatmap.xaxis.set_ticklabels(heatmap.xaxis.get_ticklabels(), rotation=45, ha='right', fontsize=fontsize)
    axes.set_ylabel('True label')
    axes.set_xlabel('Predicted label')
    axes.set_title("Confusion Matrix for predicting - " + class_label)


def multi_cm(cm):
    print("Multi Confusion matrix: ", cm)

    fig, ax = plt.subplots(5, 2, figsize=(12, 7))
    for axes, cfs_matrix, label in zip(ax.flatten(), cm,
                                       ['GC1', 'GC2', 'GC3', 'GC4', 'GC5', 'GC6',
                                        'GC7', 'GC8', 'GC9', 'GC10']):
        cm_normalized = cfs_matrix.astype('float') / cfs_matrix.sum(axis=1)[:, np.newaxis]
        print_confusion_matrix(cm_normalized, axes, label, ["N", "Y"])
    fig.suptitle('Confusion Matrix - XGBoost Independent Classifiers', fontsize=14)
    fig.tight_layout()
    plt.show()


def evaluate_model(classifier, X_train, y_train, X_test, y_test, label):
    print("Evaluation for label: ", label)
    y_pred = classify_label(X_train, y_train, X_test, classifier, label)
    # Evaluate predictions
    cm = confusion_matrix(y_test[[label]], y_pred)
    print("Confusion matrix: ", cm)
    cm_normalized = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
    # print_confusion_matrix(cm_normalized, label)

    TN = cm[0][0]
    FN = cm[0][1]
    FP = cm[1][0]
    TP = cm[1][1]
    ACC = float(TN + TP) / float(TN + FN + TP + FP)
    Precision = float(TP) / float(TP + FP)
    Recall = float(TP) / float(TP + FN)
    F1 = 2 * ((Precision * Recall) / (Precision + Recall))
    NPV = float(TN) / float(TN + FN)
    print("Self-calculated  Results:")
    print("Test Accuracy : %.4g" % ACC)
    print("Precision : %.4g" % Precision)
    print("Recall : %.4g" % Recall)
    print("F1 score : %.4g" % F1)

    return ACC, Precision, Recall, F1, cm_normalized, NPV


def save_model(param, X_train, y_train, label):
    bst = xgb.XGBClassifier(**param).fit(X_train, y_train[[label]])
    filename = 'XGBOOST' + str(label) + 'model.joblib'
    # to save the model
    joblib.dump(bst, filename)
    return filename


def load_model(filename):
    # to load the saved model
    start = time.time()
    model = joblib.load(open(filename, 'rb'))
    end = time.time()
    print("Time to load ", end - start)
    return model


def BinaryRelevance():
    classifier = OneVsRestClassifier(xgb.XGBClassifier(scale_pos_weight=5, max_depth=7))
    return classifier


def analyze_results(csv_file):
    df = pd.read_csv(csv_file)
    result1 = pd.DataFrame(columns=["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"])
    result2 = pd.DataFrame(columns=["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"])
    # Analyze True values v.s Predictions
    for i in range(11):
        percentages = []
        percentages2 = []
        for j in range(11):
            percentages.append(len(
                df.loc[df["Total_GC"] == i].loc[df.loc[df["Total_GC"] == i]["Total_PredictionsGC"] == j].index) / len(
                df.loc[df["Total_GC"] == i].index))

            percentages2.append(len(df.loc[df["Total_PredictionsGC"] == i].loc[
                                        df.loc[df["Total_PredictionsGC"] == i]["Total_GC"] == j].index) / len(
                df.loc[df["Total_PredictionsGC"] == i].index))

        result1 = result1.append(
            pd.DataFrame([percentages], columns=["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]),
            ignore_index=True)

        result2 = result2.append(
            pd.DataFrame([percentages2], columns=["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]),
            ignore_index=True)

    result1 = result1[result1.columns].astype(float)
    result2 = result2[result2.columns].astype(float)
    # print(result1)
    plt.figure(figsize=(15, 15))
    sns.set(font_scale=1.5)
    sns.heatmap(result1, annot=True, fmt='.0%')
    plt.title("Heatmap of Total GC")
    plt.xlabel("Predicted Total GC")
    plt.ylabel("Actual Total CG")
    plt.show()

    result2 = result2[result2.columns].astype(float)
    print(result2)
    plt.figure(figsize=(15, 15))
    sns.set(font_scale=1.5)
    sns.heatmap(result2, annot=True, fmt='.0%')
    plt.title("Heatmap of Total GC")
    plt.ylabel("Predicted Total GC")
    plt.xlabel("Actual Total CG")
    plt.show()

    # Bar plot
    X = [j for j in range(11)]
    # fig = make_subplots(rows=1, cols=2, shared_yaxes=True, subplot_titles=("Manipulators who reached none of the GC",
    #                                                                        "Manipulators who reached all the GC"))
    #
    # # , title="Errors in GC", labels={'x': "Total Predictions for Actual Total ", 'y':"Frequency"},
    # # , title="Errors in GC", labels={'x': "Total Predictions for Actual Total ", 'y': "Frequency"},
    # fig.add_trace(
    #     go.Bar(x=X, y=result1.loc[0], text=((result1.loc[0].round(4)) * 100).apply(lambda x: '{0:.1f}%'.format(x)),
    #            marker=dict(coloraxis="coloraxis"),
    #            name="Reached none of GC"), row=1, col=1)
    # fig.add_trace(
    #     go.Bar(x=X, y=result1.loc[10], text=((result1.loc[10].round(4)) * 100).apply(lambda x: '{0:.1f}%'.format(x)),
    #            marker=dict(coloraxis="coloraxis"),
    #            name="Reached all GC"), row=1, col=2)
    # fig.update_layout(height=600, width=800, title_text="Total of reaches", coloraxis=dict(colorscale='Bluered_r'),
    #                   showlegend=False)
    # fig.update_traces(textfont_size=12, textangle=0, textposition="outside")
    # fig.update_layout(uniformtext_minsize=10)
    # fig['layout']['xaxis']['title'] = 'Predicted number of reachable GC'
    # fig['layout']['xaxis2']['title'] = 'Predicted number of reachable GC'
    # fig['layout']['yaxis']['title'] = 'Number of manipulators (%)'
    # fig.show()

    # fig = go.Figure()
    # fig.add_trace(go.Bar(x=X,
    #                      y=result1.loc[0],
    #                      name='Reached none of GC',
    #                      text=((result1.loc[0].round(4)) * 100).apply(lambda x: '{0:.1f}%'.format(x)),
    #                      marker_color='rgb(55, 83, 109)'
    #                      ))
    # fig.add_trace(go.Bar(x=X,
    #                      y=result1.loc[10],
    #                      name='Reached all GC',
    #                      text=((result1.loc[10].round(4)) * 100).apply(lambda x: '{0:.1f}%'.format(x)),
    #                      marker_color='rgb(26, 118, 255)'
    #                      ))
    #
    # fig.update_layout(
    #     title='Total reaches of manipulators',
    #     xaxis= dict(
    #         title='Predicted number of reachable GC',
    #         titlefont_size=16,
    #         tickfont_size=14,
    #     ),
    #     yaxis=dict(
    #         title='Number of manipulators (%)',
    #         titlefont_size=16,
    #         tickfont_size=14,
    #     ),
    #     legend=dict(
    #         x=0,
    #         y=1.0,
    #         bgcolor='rgba(255, 255, 255, 0)',
    #         bordercolor='rgba(255, 255, 255, 0)'
    #     ),
    #     barmode='group',
    #     bargap=0.15,  # gap between bars of adjacent location coordinates.
    #     bargroupgap=0.1  # gap between bars of the same location coordinate.
    # )
    # fig.show()

    how_manyP = ['0', '1', '2',
                 '3', '4', '5',
                 '6', '7', '8',
                 '9', '10', '0', '1', '2',
                 '3', '4', '5',
                 '6', '7', '8',
                 '9', '10']
    true0 = result1.loc[10].round(4)
    true10 = result1.loc[0].round(4)
    trues = true10.append(true0)
    print(trues)
    how_manyT = ['Actual reached 0 GC', 'Actual reached 0 GC', 'Actual reached 0 GC',
                 'Actual reached 0 GC','Actual reached 0 GC', 'Actual reached 0 GC',
                 'Actual reached 0 GC', 'Actual reached 0 GC', 'Actual reached 0 GC',
                 'Actual reached 0 GC', 'Actual reached 0 GC', 'Actual reached 10 GC',
                 'Actual reached 10 GC', 'Actual reached 10 GC', 'Actual reached 10 GC',
                 'Actual reached 10 GC','Actual reached 10 GC','Actual reached 10 GC',
                 'Actual reached 10 GC', 'Actual reached 10 GC', 'Actual reached 10 GC',
                 'Actual reached 10 GC']

    fig = px.bar(x=how_manyT, y=trues, color=how_manyP, color_discrete_sequence=px.colors.sequential.algae,
                 text_auto=(trues.apply(lambda x: '{0:.0f}%'.format(x))), width=600)
    fig.update_traces(textposition='inside')
    fig.update_layout(uniformtext_minsize=8, uniformtext_mode='hide')
    fig.update_layout(legend=dict(
        title="Total GC Predicted"))
    fig.update_layout(
        title='True reaches vs. Total predicted',
        xaxis=dict(
            # visible=False, showticklabels=True,
            # title='Predicted number of reachable GC',
            titlefont_size=16,
            tickfont_size=14,
        ),
        yaxis=dict(
            title='Number of manipulators (%)',
            titlefont_size=16,
            tickfont_size=14,
        ))
    fig.update_traces(width=0.5)
    fig.show()
    fig.write_image("barplot.png")


def prepare_input(joint_types, joint_axis, links):
    sample = pd.DataFrame(columns=['Link2 length', 'Link3 length', 'Link4 length', 'Link5 length',
                                   'Link6 length', 'Joint2 type_pris', 'Joint2 type_roll', 'Joint2 axis_z',
                                   'Joint3 type_pris', 'Joint3 type_roll', 'Joint3 axis_y',
                                   'Joint3 axis_z', 'Joint4 type_pris', 'Joint4 type_roll',
                                   'Joint4 axis_y', 'Joint4 axis_z', 'Joint5 type_pris',
                                   'Joint5 type_roll', 'Joint5 axis_y', 'Joint5 axis_z',
                                   'Joint6 type_pris', 'Joint6 type_roll', 'Joint6 axis_y',
                                   'Joint6 axis_z'])

    for i in range(1, len(joint_types)):
        sample.at[0, 'Link' + str(i + 1) + ' length'] = links[i]
        if joint_types[i] == 'pris':
            sample.at[0, 'Joint' + str(i + 1) + ' type_pris'] = 1
            sample.at[0, 'Joint' + str(i + 1) + ' type_roll'] = 0
        elif joint_types[i] == 'roll':
            sample.at[0, 'Joint' + str(i + 1) + ' type_pris'] = 0
            sample.at[0, 'Joint' + str(i + 1) + ' type_roll'] = 1
        elif joint_types[i] == 'pitch':
            sample.at[0, 'Joint' + str(i + 1) + ' type_pris'] = 0
            sample.at[0, 'Joint' + str(i + 1) + ' type_roll'] = 0

        # Joint2 axis can be either z or y
        if joint_axis[i] == 'x':
            print(joint_axis[i])
            sample.at[0, 'Joint' + str(i + 1) + ' axis_y'] = 0
            sample.at[0, 'Joint' + str(i + 1) + ' axis_z'] = 0
        elif joint_axis[i] == 'y':
            print(joint_axis[i])
            sample.at[0, 'Joint' + str(i + 1) + ' axis_y'] = 1
            sample.at[0, 'Joint' + str(i + 1) + ' axis_z'] = 0
        elif joint_axis[i] == 'z':
            print(joint_axis[i])
            sample.at[0, 'Joint' + str(i + 1) + ' axis_y'] = 0
            sample.at[0, 'Joint' + str(i + 1) + ' axis_z'] = 1

    ordered_sample = sample[['Link2 length', 'Link3 length', 'Link4 length', 'Link5 length',
                             'Link6 length', 'Joint2 type_pris', 'Joint2 type_roll', 'Joint2 axis_z',
                             'Joint3 type_pris', 'Joint3 type_roll', 'Joint3 axis_y',
                             'Joint3 axis_z', 'Joint4 type_pris', 'Joint4 type_roll',
                             'Joint4 axis_y', 'Joint4 axis_z', 'Joint5 type_pris',
                             'Joint5 type_roll', 'Joint5 axis_y', 'Joint5 axis_z',
                             'Joint6 type_pris', 'Joint6 type_roll', 'Joint6 axis_y',
                             'Joint6 axis_z']]

    return ordered_sample


def inference(sample):
    targets = ['GC' + str(i + 1) for i in range(10)]
    res = pd.DataFrame(columns=['PredictionsGC1', 'PredictionsGC2', 'PredictionsGC3',
                                'PredictionsGC4', 'PredictionsGC5', 'PredictionsGC6',
                                'PredictionsGC7', 'PredictionsGC8', 'PredictionsGC9',
                                'PredictionsGC10'])
    for label in targets:
        model = load_model('XGBOOST' + label + 'model.joblib')
        pred = model.predict(sample)
        res.at[0, 'Predictions' + label] = pred.item(0)

    total = (res[['PredictionsGC1', 'PredictionsGC2', 'PredictionsGC3',
                  'PredictionsGC4', 'PredictionsGC5', 'PredictionsGC6',
                  'PredictionsGC7', 'PredictionsGC8', 'PredictionsGC9',
                  'PredictionsGC10']] == 1).sum(axis=1).reset_index(drop=True)
    print(res)
    print(total)

    if total[0] < 7:
        return False
    else:
        return True


def multi_models(X, y, predict, BR):
    X_train, X_test, X_val, y_train, y_test, y_val = split_data(X, y)
    targets = list(y_train.columns.values)
    targets.remove("Total_GC")
    print(targets)

    if BR:
        classifier = BinaryRelevance()
        classifier.fit(X_train, y_train.drop(columns="Total_GC"))
        preds = classifier.predict(X_test)

        for i in range(10):
            col = [row[i] for row in preds]
            y_test.insert(11, 'PredictionsGC' + str(i + 1), col)

        y_test['Total_PredictionsGC'] = (y_test[['PredictionsGC1', 'PredictionsGC2', 'PredictionsGC3',
                                                 'PredictionsGC4', 'PredictionsGC5', 'PredictionsGC6',
                                                 'PredictionsGC7', 'PredictionsGC8', 'PredictionsGC9',
                                                 'PredictionsGC10']] == 1).sum(axis=1)

        y_test.to_csv('BRPreds.csv')
    else:
        # Predict and save result
        if predict:
            cms = []
            for label in targets:
                filename = 'XGBOOST' + str(label) + 'model.joblib'
                classifer = load_model(filename)
                preds = classify_label(X_train, y_train, X_test, classifer, label)
                y_test.insert(11, 'Predictions' + label, preds)
                print(y_test.columns)
                ACC, Precision, Recall, F1, cm, NPV = evaluate_model(classifer, X_train, y_train, X_test, y_test, label)
                cms.append(cm)

            multi_cm(cms)
            y_test['Total_PredictionsGC'] = (y_test[['PredictionsGC1', 'PredictionsGC2', 'PredictionsGC3',
                                                     'PredictionsGC4', 'PredictionsGC5', 'PredictionsGC6',
                                                     'PredictionsGC7', 'PredictionsGC8', 'PredictionsGC9',
                                                     'PredictionsGC10']] == 1).sum(axis=1)
            print(y_test.columns)
            print(y_test[["Total_GC", "Total_PredictionsGC"]])
            y_test.to_csv('Preds.csv')
            print("Accuracy for total predictions",
                  accuracy_score(y_test[["Total_GC"]], y_test[["Total_PredictionsGC"]]))


        # Train and Save models
        else:
            cms = []
            for label in targets:
                clf, best_params = train_per_label(X_train, y_train, X_test, y_test, True, label)
                ACC, Precision, Recall, F1, cm, NPV = evaluate_model(clf, X_train, y_train, X_test, y_test, label)
                cms.append(cm)
                filename = save_model(best_params, X_train, y_train, label)
                print("Model saved as: ", filename)
            multi_cm(cms)


def cross_validate(X, y, targets, cv):
    for label in targets:
        filename = 'XGBOOST' + str(label) + 'model.joblib'
        classifer = load_model(filename)
        npvs = []
        ppvs = []
        for i in range(cv):
            X_train, X_test, y_train, y_test = split_data(X, y, seed=i * 24, val=False)
            preds = classify_label(X_train, y_train, X_test, classifer, label)
            y_test.insert(11, 'Predictions' + label, preds)
            print(y_test.columns)
            ACC, Precision, Recall, F1, cm, NPV = evaluate_model(classifer, X_train, y_train, X_test, y_test, label)
            print(cm)
            npvs.append(NPV)
            ppvs.append(Precision)

        print("NPVS FOR LABEL " + label, npvs)
        print("Average NPV for label: " + label, mean(npvs))
        print("PPVS FOR LABEL " + label, ppvs)
        print("Average PPV for label: " + label, mean(ppvs))


if __name__ == '__main__':
    X, y = preprocessing('6dof_dataset.csv')
    multi_models(X, y, predict=True, BR=False)
    # analyze_results('Preds.csv')

    # joint_types = ['roll', 'pitch', 'pris', 'pris', 'pris', 'pris']
    # joint_axis = ['x', 'y', 'z', 'z', 'z', 'z']
    # links = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    # sample = prepare_input(joint_types, joint_axis, links)
    #
    # promising = inference(sample)
    # print(bool)

    # X, y = preprocessing('6dof_dataset.csv')
    # targets = ['GC' + str(i + 1) for i in range(10)]
    # cross_validate(X, y, targets, 5)
